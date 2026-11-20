#ifndef COMMAND_TASK_H
#define COMMAND_TASK_H

#include <Arduino_FreeRTOS.h>
#include "commands.h"
#include "../behaviors/line_follower.h"
#include "../behaviors/calibration.h"

// Tarea que atiende los comandos del PC. Debe tener menos prioridad que el seguidor
// (line_follower::PRIORITY) para poder detenerlo.
namespace commands
{
    const TickType_t POLL_PERIOD = pdMS_TO_TICKS(90);

    struct Components
    {
        Logger &logger;
        line_follower::Components &line_follower;
        calibration::Components &calibration;
        configSTACK_DEPTH_TYPE line_follower_stack; // pila de cada recorrido que empieza g
    };

    // Limites de los valores de p (Kp, Ki, Kd) y b (base, curva, interior).
    const float GAIN_MIN[3] = {0, 0, 0};
    const float GAIN_MAX[3] = {10, 10, 10};
    const float SPEED_MIN[3] = {0, 0, -Movement::MAX_SPEED};
    const float SPEED_MAX[3] = {Movement::MAX_SPEED, Movement::MAX_SPEED, Movement::MAX_SPEED};

    bool in_range(const float (&min_value)[3], const float (&max_value)[3])
    {
        for (size_t i = 0; i < 3; i++)
        {
            if (values[i] < min_value[i] || values[i] > max_value[i])
                return false;
        }
        return true;
    }

    // Aplica los numeros de p o b. Devuelve false si alguno esta fuera de rango.
    bool apply_values(bool gains)
    {
        if (!(gains ? in_range(GAIN_MIN, GAIN_MAX) : in_range(SPEED_MIN, SPEED_MAX)))
            return false;

        const line_follower::Speeds speeds = {(int)lround(values[0]), (int)lround(values[1]), (int)lround(values[2])};
        // Seccion critica: el seguidor no puede leer los valores a medio escribir.
        taskENTER_CRITICAL();
        if (gains)
            line_follower::pid.SetTunings(values[0], values[1], values[2]);
        else
            line_follower::speeds = speeds;
        taskEXIT_CRITICAL();
        return true;
    }

    // Formato CSV: pid,<kp>,<ki>,<kd> o speeds,<base>,<curva>,<interior>
    void print_values(Logger &logger, bool gains)
    {
        if (gains)
        {
            logger.print("pid,");
            logger.print_decimal(line_follower::pid.GetKp());
            logger.print(',');
            logger.print_decimal(line_follower::pid.GetKi());
            logger.print(',');
            logger.print_decimal(line_follower::pid.GetKd());
            logger.println("");
            return;
        }
        logger.print("speeds,");
        logger.print(line_follower::speeds.base);
        logger.print(',');
        logger.print(line_follower::speeds.curve);
        logger.print(',');
        logger.println(line_follower::speeds.inner_min);
    }

    // Formato CSV: thresholds,<s0>,<s1>,<s2> y despues los valores de p y b.
    void print_header(Components &hw)
    {
        hw.logger.print("thresholds");
        for (size_t i = 0; i < 3; i++)
        {
            hw.logger.print(',');
            hw.logger.print(hw.line_follower.sensors.threshold(i));
        }
        hw.logger.println("");
        print_values(hw.logger, true);
        print_values(hw.logger, false);
    }

    // Formato CSV: run,<ms>,<estado>
    void print_run(Logger &logger, const char *status)
    {
        logger.print("run,");
        logger.print(millis());
        logger.print(',');
        logger.println(status);
    }

    // Respuesta a p y b: los valores en uso, o pid,error / speeds,error si se rechazaron.
    void handle_values(Logger &logger, uint8_t requests)
    {
        const bool gains = values_command == GAINS;
        const bool rejected = (requests & REQUEST_VALUES_INVALID) ||
                              ((requests & REQUEST_VALUES_SET) && !apply_values(gains));
        if (rejected)
            logger.println(gains ? "pid,error" : "speeds,error");
        if (requests & (REQUEST_VALUES_SET | REQUEST_VALUES_QUERY))
            print_values(logger, gains);
    }

    // Calibra aqui mismo (unos 4 s sin atender otros comandos). Con un recorrido en
    // marcha responde calibration,ignored: hay que detenerlo antes con x.
    void handle_calibration(Components &hw)
    {
        if (line_follower::running())
        {
            hw.logger.println("calibration,ignored");
            return;
        }
        hw.logger.println("calibration,start");
        calibration::calibrate_line_sensors(hw.calibration);
        hw.logger.println("calibration,complete");
    }

    void handle(Components &hw, uint8_t requests)
    {
        if (requests & REQUEST_HEADER)
            print_header(hw);
        if (requests & (REQUEST_VALUES_SET | REQUEST_VALUES_QUERY | REQUEST_VALUES_INVALID))
            handle_values(hw.logger, requests);
        // Antes que REQUEST_RUN: "xg" detiene el recorrido y empieza otro.
        if (requests & REQUEST_HALT)
            print_run(hw.logger, line_follower::stop() ? "stop" : "ignored");
        if (requests & REQUEST_RUN)
            print_run(hw.logger, line_follower::start(hw.line_follower, hw.line_follower_stack) ? "start" : "ignored");
        if (requests & REQUEST_CALIBRATION)
            handle_calibration(hw);
    }

    // parameters: Components.
    void task(void *parameters)
    {
        Components &hw = *static_cast<Components *>(parameters);
        for (;;)
        {
            handle(hw, poll(hw.logger));
            hw.logger.flush_events();
            vTaskDelay(POLL_PERIOD);
        }
    }
}

#endif // COMMAND_TASK_H
