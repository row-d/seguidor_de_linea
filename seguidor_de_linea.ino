#include <RobotComponents.h>

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

Robot<3> robot_controller{
    mov,
    line_sensors,
    color_sensor,
    claw_l,
    buzzer,
    turn_leds,
    obstacle_sensor};

void setup()
{
    Serial.begin(9600);
    robot_controller.setup();
    robot_controller.start();
}

void loop() {}
