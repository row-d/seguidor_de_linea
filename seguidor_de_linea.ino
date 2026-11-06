#include <RobotComponents.h>
#include "bluetooth_config.h"

void read_rgb_sensor(RgbColor &color)
{
    color = {0, 0, 0};
}

// Modulo: E1, M1 para la izquierda; E2, M2 para la derecha.
Movement mov{2, 3, 4, 5};
const int line_sensor_pins[] = {A0, A1, A2};
const int line_thresholds[] = {110, 120, 100};
LineSensors<3> line_sensors{line_sensor_pins, line_thresholds};
ColorSensor color_sensor{read_rgb_sensor, {255, 0, 0}, 30};
Claw claw_l{8, 20, 90};
Claw claw_r{9, 20, 90};
Buzzer buzzer{10};
const int turn_led_pins[] = {11, 12};
LedArray<2> turn_leds{turn_led_pins};
UltrasonicSensor obstacle_sensor{7, 6};
// Serial1: Bluetooth. Serial: USB.
Logger logger{Serial1, BLUETOOTH_BAUD};
Hc05 bluetooth{Serial1, Serial};

Robot<3> robot_controller{
    mov,
    line_sensors,
    color_sensor,
    claw_l,
    buzzer,
    turn_leds,
    obstacle_sensor,
    logger};

const bool calibration_mode = false;
const Route preferred_route = ROUTE_RIGHT;

void setup()
{
    if (BLUETOOTH_SETUP_MODE)
    {
        mov.setup();
        Serial.begin(9600);
        bluetooth.setup();
        const bool configured = bluetooth.configure(BLUETOOTH_NAME, BLUETOOTH_BAUD);
        Serial.println(configured ? "bluetooth,ok" : "bluetooth,error");
        return;
    }
    // Serial.begin(9600);
    robot_controller.load_calibration();
    robot_controller.setup();
    if (calibration_mode)
        robot_controller.calibrate();
    else
        robot_controller.start(preferred_route);
}

void loop() {}
