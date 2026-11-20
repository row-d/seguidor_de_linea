#ifndef CALIBRATION_TASK_H
#define CALIBRATION_TASK_H

#include <Arduino_FreeRTOS.h>
#include "../components/line_sensors.h"
#include "../components/wheel_sensor.h"
#include "../components/movement.h"
#include "../components/logger.h"

// Tareas de calibracion: imprimen los valores que hay que copiar en el sketch.
namespace calibration
{
    struct Components
    {
        LineSensors<3> &line_sensors;
        WheelSensor<2> &wheel_sensors;
        Movement &wheels;
        Logger &logger;
    };

    void print(Logger &logger, const LineSensors<3>::Calibration &cal)
    {
        for (size_t i = 0; i < 3; i++)
        {
            logger.print("sensor ");
            logger.print(i);
            logger.print(": min=");
            logger.print(cal.min_read[i]);
            logger.print(" max=");
            logger.print(cal.max_read[i]);
            logger.print(" threshold=");
            logger.println(cal.threshold(i));
        }
    }

    // Gira sobre su eje a la derecha midiendo el fondo y la linea con cada sensor, y
    // aplica los umbrales resultantes. Solo desde una tarea: espera con vTaskDelay.
    void calibrate_line_sensors(Components &hw)
    {
        const unsigned long CALIBRATION_MS = 4000;
        const unsigned long PRINT_EVERY_MS = 500;
        const int CALIBRATION_SPEED = 110;

        LineSensors<3>::Calibration cal;

        hw.logger.println("calibrando...");
        hw.wheels.drive(CALIBRATION_SPEED, -CALIBRATION_SPEED);

        const unsigned long start_time = millis();
        unsigned long last_print = start_time;
        while (millis() - start_time < CALIBRATION_MS)
        {
            hw.line_sensors.sample_calibration(cal);
            if (millis() - last_print >= PRINT_EVERY_MS)
            {
                last_print = millis();
                print(hw.logger, cal);
            }
            vTaskDelay(1);
        }
        hw.wheels.stop();

        hw.logger.println("calibracion final:");
        print(hw.logger, cal);
        hw.line_sensors.apply_calibration(cal);
    }

    // parameters: Components.
    void line_sensors_task(void *parameters)
    {
        Components &hw = *static_cast<Components *>(parameters);
        calibrate_line_sensors(hw);
        hw.logger.println("copia los thresholds en thresholds[] y pon CALIBRATE en 0");
        vTaskDelay(pdMS_TO_TICKS(500));
        vTaskDelete(nullptr);
    }

    // Con el robot levantado (ruedas en el aire). Sube el PWM de las dos ruedas a la vez
    // desde parado: el primer PWM con el que el encoder ve giro es el minimo de esa rueda.
    // Por encima del minimo las rpm crecen casi en linea recta con el PWM; la pendiente
    // (rpm por punto de PWM) dice cuanto corre cada rueda, y la mas rapida se escala para
    // igualar a la lenta. parameters: Components.
    void wheels_task(void *parameters)
    {
        const int PWM_START = 30;
        const int PWM_STEP = 5;
        const unsigned long STEP_MS = 500; // > 300 ms: rpm() da 0 si no hubo flancos en ese tiempo
        const int SLOPE_MARGIN = 20;       // la pendiente se mide desde max(minimos) + margen

        Components &hw = *static_cast<Components *>(parameters);
        Logger &logger = hw.logger;
        int min_pwm[2] = {0, 0}; // {izquierda, derecha}
        float rpm_sum[2] = {0, 0};
        long pwm_sum[2] = {0, 0};

        logger.println("calibrando ruedas (robot levantado)...");
        logger.println("pwm\trpm_l\trpm_r");
        for (int pwm = PWM_START; pwm <= Movement::MAX_SPEED; pwm += PWM_STEP)
        {
            hw.wheels.drive(pwm, pwm);
            vTaskDelay(pdMS_TO_TICKS(STEP_MS));

            float rpm[2];
            for (size_t i = 0; i < 2; i++)
            {
                rpm[i] = hw.wheel_sensors.rpm(i);
                if (min_pwm[i] == 0 && rpm[i] > 0)
                    min_pwm[i] = pwm;
            }

            logger.print(pwm);
            logger.print("\t");
            logger.print(rpm[0]);
            logger.print("\t");
            logger.println(rpm[1]);

            if (min_pwm[0] && min_pwm[1] && pwm >= max(min_pwm[0], min_pwm[1]) + SLOPE_MARGIN)
            {
                for (size_t i = 0; i < 2; i++)
                {
                    rpm_sum[i] += rpm[i];
                    pwm_sum[i] += pwm - min_pwm[i];
                }
            }
        }
        hw.wheels.stop();

        if (pwm_sum[0] == 0 || pwm_sum[1] == 0 || rpm_sum[0] <= 0 || rpm_sum[1] <= 0)
        {
            logger.println("una rueda no giro o no se leyo: revisa motor y encoder (pines 20 y 21)");
            vTaskDelete(nullptr);
        }

        const float slope_l = rpm_sum[0] / pwm_sum[0];
        const float slope_r = rpm_sum[1] / pwm_sum[1];

        logger.println("copia estos valores y pon CALIBRATE_WHEELS en 0:");
        logger.print("const int MIN_PWM_L = ");
        logger.print(min_pwm[0]);
        logger.println(";");
        logger.print("const int MIN_PWM_R = ");
        logger.print(min_pwm[1]);
        logger.println(";");
        logger.print("const float WHEEL_TRIM_L = ");
        logger.print_decimal(slope_l > slope_r ? slope_r / slope_l : 1.0f);
        logger.println("f;");
        logger.print("const float WHEEL_TRIM_R = ");
        logger.print_decimal(slope_r > slope_l ? slope_l / slope_r : 1.0f);
        logger.println("f;");

        vTaskDelete(nullptr);
    }
}

#endif
