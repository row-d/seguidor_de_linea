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

// Velocidades (PWM, 0..255). Se cambian desde el PC con el comando b y se guardan en
// EEPROM (ver speed_settings.h); estos valores solo valen si no hay otros guardados.
struct SpeedSettings
{
    // Velocidad de avance sobre la linea.
    int base;
    // Velocidad del lado que empuja en un giro.
    int turn;
    // PWM minimo con el que una rueda gira de verdad (ver Movement::set_min_speed).
    int min;
};
// La tarea de telemetria las cambia; las demas solo las leen.
static SpeedSettings speed_settings = {130, 230, 125};

// Giro tipo tanque. Se cambia desde el PC con el comando t y se guarda en EEPROM (ver
// speed_settings.h); este valor solo vale si no hay otro guardado.
// true: gira sobre su eje, con un lado hacia adelante y el otro hacia atras. Es el giro
// mas cerrado, pero arrastra las 4 ruedas de lado: con poca bateria los motores no
// tienen fuerza y el robot se queda quieto.
// false: gira con un lado detenido y el otro avanzando, en un arco corto. Pide menos
// fuerza.
static bool tank_turn = true;

// Velocidad de cada lado para girar hacia direction: -1 izquierda, 1 derecha.
inline void turn_speeds(int direction, int &left, int &right)
{
    const int outer = speed_settings.turn;
    const int inner = tank_turn ? -outer : 0;
    left = direction < 0 ? inner : outer;
    right = direction < 0 ? outer : inner;
}

// Ganancias del PID si no hay otras guardadas en EEPROM (ver pid_gains.h).
// Con Kp 0.25 y error 500 (linea entre el centro y un lado) la correccion es 125:
// la rueda interior se detiene y la exterior va al maximo.
const float DEFAULT_PID_KP = 0.25;
const float DEFAULT_PID_KI = 0.0;
// PID_v1 divide Kd entre el periodo en segundos (0.005 / 0.025 = 0.2 por muestra).
const float DEFAULT_PID_KD = 0.005;

// Definidos en start.h y line_follower.h.
struct RunState;
struct WheelSpeeds;

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
    // Motivo y hora de la ultima vez que el programa detuvo el robot (nullptr: ninguna
    // en este recorrido). Se guarda aunque no haya registro ni Bluetooth, para poder
    // consultarlo despues. Lo escribe la tarea de control con note_stop().
    const char *last_stop_reason = nullptr;
    unsigned long last_stop_ms = 0;
    // Resultado de la ultima calibracion, que envia la tarea de telemetria.
    int calibration_minimums[N];
    int calibration_maximums[N];

    void wait_ms(uint32_t duration_ms)
    {
        vTaskDelay(pdMS_TO_TICKS(duration_ms));
    }

    // Tarea de control (start.h): maniobras y pasos del ciclo.
    void run();
    CrossResult cross_horizontal_line();
    void show_route(Route route);
    void take_branch(RunState &state, Route side, uint32_t min_turn_ms);
    void wait_calibration(RunState &state);
    bool pass_start_line(RunState &state, bool line_detected);
    void end_fork_wait(RunState &state);
    void check_line_lost(RunState &state, bool line_detected);
    bool see_fork(RunState &state);
    bool follow_route(RunState &state, bool fork_seen);
    bool go_back_to_fork(RunState &state, bool fork_seen);
    bool remove_obstacle(RunState &state, bool line_detected);
    void send_sample(int line_error, bool line_detected, const WheelSpeeds &speeds);
    void log_header();
    void log_calibration_report();
    void log_gains();
    void log_speeds();
    void log_last_stop();
    void note_stop(const char *reason);
    bool set_speeds(int base, int turn, int min);
    bool set_gains(float kp, float ki, float kd);
    void read_gains(float &kp, float &ki, float &kd);
    void request_calibration();
    void run_calibration();
    void start_services();
    void turn_to(Route side, uint32_t min_turn_ms);
    void end_run(const char *reason);
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
    void load_speeds();
    bool toggle_tank_turn();
    void debug();
};

#endif
