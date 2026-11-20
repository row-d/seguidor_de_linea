#include <RobotComponents.h>
#include "bluetooth_config.h"

// 1: gira sobre su eje midiendo los sensores de linea e imprime los valores de thresholds.
#define CALIBRATE 0
// 1: con el robot levantado, mide con los encoders el PWM minimo y lo que corre cada
// rueda, e imprime los valores de MIN_PWM_L/R y WHEEL_TRIM_L/R. Tiene prioridad sobre CALIBRATE.
#define CALIBRATE_WHEELS 0
// 1: atiende los comandos de scripts/console.ps1 (g, x, c, p, b, r, s) por el puerto que
// elige COMMANDS_OVER_USB en bluetooth_config.h. Sin efecto con CALIBRATE o CALIBRATE_WHEELS.
#define COMMANDS 0

// Valores que imprimen las calibraciones.
const int thresholds[3] = {457, 275, 261};
const int MIN_PWM_L = 100; // PWM minimo con el que arranca cada rueda
const int MIN_PWM_R = 150;
const float WHEEL_TRIM_L = 1.0f;    // escala el PWM por encima del minimo para que las dos
const float WHEEL_TRIM_R = 0.6643f; // ruedas corran igual con la misma velocidad pedida

const configSTACK_DEPTH_TYPE TASK_STACK = 8192 / 6; // RAM de la Mega / 6

Claw claw_l{9, 90, 0, 5, 15};
UltrasonicSensor obstacle_sensor{7, 6};
Hc05 bluetooth{Serial1, Serial};
Logger logger{COMMANDS_OVER_USB ? Serial : Serial1, BLUETOOTH_BAUD};
Movement wheels{2, 3, 4, 5};
const int line_pins[3] = {A0, A1, A2}; // {izquierdo, central, derecho}
LineSensors<3> line_sensors{line_pins, thresholds};
Buzzer buzzer{10};
const int turn_led_pins[2] = {11, 12}; // {izquierdo, derecho}
LedArray<2> turn_leds{turn_led_pins};
// Encoders DFRobot SEN0038. Pines con interrupcion: 2 y 3 son de los motores y
// 18 y 19 de Serial1 (HC-05).
const int wheel_sensor_pins[2] = {20, 21}; // {izquierda, derecha}
WheelSensor<2> wheel_sensors{wheel_sensor_pins};

calibration::Components calibration_components{line_sensors, wheel_sensors, wheels, logger};
line_follower::Components line_follower_components{line_sensors, wheels, turn_leds, buzzer, logger};
collision::Components collision_components{obstacle_sensor, claw_l, logger};
commands::Components commands_components{logger, line_follower_components, calibration_components, TASK_STACK};

void setup()
{
    claw_l.setup();
    obstacle_sensor.setup();
    logger.setup();
    wheels.setup();
    wheel_sensors.setup();
    line_sensors.setup();
    turn_leds.setup();
    buzzer.setup();
    wheels.set_compensation(MIN_PWM_L, WHEEL_TRIM_L, MIN_PWM_R, WHEEL_TRIM_R);

#if CALIBRATE_WHEELS
    xTaskCreate(calibration::wheels_task, "calibrate_wheels", TASK_STACK, &calibration_components, 1, nullptr);
#elif CALIBRATE
    xTaskCreate(calibration::line_sensors_task, "calibrate", TASK_STACK, &calibration_components, 1, nullptr);
#else
    line_follower::start(line_follower_components, TASK_STACK);
    xTaskCreate(collision::task, "collision", TASK_STACK, &collision_components, 1, nullptr);
#if COMMANDS
    xTaskCreate(commands::task, "commands", TASK_STACK / 2, &commands_components, 1, nullptr);
#endif
#endif
}

// Vacio: la logica corre en las tareas de FreeRTOS.
void loop() {}
