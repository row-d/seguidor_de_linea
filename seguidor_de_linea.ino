#include <RobotComponents.h>

void read_rgb_sensor(RgbColor &color)
{
    // Reemplazar con la lectura del sensor RGB.
    color = {0, 0, 0};
}

Movement mov{2, 3, 4, 5};
LineSensors<3> line_sensors{(const int[]){0, 1, 2}};
ColorSensor color_sensor{read_rgb_sensor, {255, 0, 0}, 30};
Claw claw{8, 20, 90};
Buzzer buzzer{6};
const int turn_led_pins[] = {10, 11};
LedArray<2> turn_leds{turn_led_pins};
UltrasonicSensor obstacle_sensor{12, 13};

Robot<3> robot_controller{
    mov,
    line_sensors,
    color_sensor,
    claw,
    buzzer,
    turn_leds,
    obstacle_sensor
};

void setup()
{
    Serial.begin(9600);
    robot_controller.setup();
    robot_controller.start();
}

void loop() {}
