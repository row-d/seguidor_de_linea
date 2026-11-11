#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include "../components/component.h"
#include "../components/movement.h"
#include "../components/line_sensors.h"
#include "../components/color_sensor.h"
#include "../components/claw.h"
#include "../components/buzzer.h"
#include "../components/led_array.h"
#include "../components/ultrasonic_sensor.h"
#include "../components/logger.h"
#include "route_memory.h"

const int BASE_SPEED = 130;
// Girar sobre su eje arrastra las 4 ruedas de lado: con 185 el robot no se movia.
const int TURN_SPEED = 230;

// Ganancias del PID si no hay otras guardadas en EEPROM (ver pid_gains.h).
// Con Kp 0.25 y error 500 (linea entre el centro y un lado) la correccion es 125:
// la rueda interior se detiene y la exterior va al maximo.
const float DEFAULT_PID_KP = 0.25;
const float DEFAULT_PID_KI = 0.0;
// PID_v1 divide Kd entre el periodo en segundos (0.005 / 0.025 = 0.2 por muestra).
const float DEFAULT_PID_KD = 0.005;

enum CrossResult
{
    CROSS_CONTINUE,
    CROSS_FINISH,
    CROSS_FORK
};

template <size_t N>
class Robot : public Component
{
public:
private:
    Movement &movement;
    LineSensors<N> &line_sensors;
    ColorSensor &color_sensor;
    Claw &claw;
    Buzzer &buzzer;
    LedArray<2> &leds;
    UltrasonicSensor &obstacle_sensor;
    Logger &logger;
    Route preferred_route = ROUTE_LEFT;
    RouteMemory<8> route_memory;

    // Lo que la tarea de control publica en cada ciclo para la tarea de telemetria.
    struct TelemetrySample
    {
        unsigned long ms;
        int raw[N];
        int error;
        int correction;
        int speed_l;
        int speed_r;
        bool line_detected;
    };

    enum CalibrationState : uint8_t
    {
        CALIBRATION_IDLE,
        CALIBRATION_RUNNING,
        CALIBRATION_DONE
    };

    // Cola de 1 elemento: solo interesa la muestra mas reciente.
    QueueHandle_t telemetry_samples = nullptr;
    // nullptr si no hay recorrido en marcha: aun no se pidio o ya termino.
    TaskHandle_t control_task = nullptr;
    TaskHandle_t calibration_task = nullptr;
    // La tarea de telemetria la activa; la de control la atiende entre dos ciclos.
    volatile bool calibration_requested = false;
    volatile CalibrationState calibration_state = CALIBRATION_IDLE;
    // Ganancias del PID. Las cambia la tarea de telemetria con set_gains(); la de
    // control las copia con read_gains() cuando gains_changed es true.
    float pid_kp = DEFAULT_PID_KP;
    float pid_ki = DEFAULT_PID_KI;
    float pid_kd = DEFAULT_PID_KD;
    volatile bool gains_changed = false;
    // Resultado de la ultima calibracion, que envia la tarea de telemetria.
    int calibration_minimums[N];
    int calibration_maximums[N];

    void wait_ms(uint32_t duration_ms)
    {
        vTaskDelay(pdMS_TO_TICKS(duration_ms));
    }

    CrossResult cross_horizontal_line();
    void show_route(Route route);
    void log_header();
    void log_calibration_report();
    void log_gains();
    bool set_gains(float kp, float ki, float kd);
    void read_gains(float &kp, float &ki, float &kd);
    void request_calibration();
    void run_calibration();
    void start_services();
    void turn_toward(Route side, uint32_t min_turn_ms);
    void halt(const char *reason);
    void remove_object();

public:
    Robot(Movement &movement_controller,
          LineSensors<N> &sensors,
          ColorSensor &color,
          Claw &robot_claw,
          Buzzer &robot_buzzer,
          LedArray<2> &robot_turn_leds,
          UltrasonicSensor &robot_obstacle_sensor,
          Logger &robot_logger)
        : movement(movement_controller),
          line_sensors(sensors),
          color_sensor(color),
          claw(robot_claw),
          buzzer(robot_buzzer),
          leds(robot_turn_leds),
          obstacle_sensor(robot_obstacle_sensor),
          logger(robot_logger) {}

    void setup()
    {
        Component *components[] = {&logger, &movement, &line_sensors, &color_sensor,
                                   &claw, &buzzer, &leds, &obstacle_sensor};
        for (Component *component : components)
            component->setup();
        // Tareas de telemetria y calibracion: existen siempre, aunque no haya recorrido.
        start_services();
    }

    void set_route(Route route)
    {
        preferred_route = route;
    }

    // TASKS
    bool start();
    void stop();
    void load_calibration();
    void load_gains();
    void debug();
};

#endif
