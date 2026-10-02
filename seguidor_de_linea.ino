#include "components/robot.h"

void read_rgb_sensor(RgbColor &color)
{
    // Reemplazar con la lectura del sensor RGB.
    color = {0, 0, 0};
}

Movement mov{10, 6, 12, 7, 13, 8, 11, 9};
const int line_sensor_pins[] = {5};
LineSensors line_sensors{line_sensor_pins, sizeof(line_sensor_pins) / sizeof(line_sensor_pins[0])};
ColorSensor color_sensor{read_rgb_sensor, {255, 0, 0}, 30};
Claw claw{3, 20, 90};

const char route[] = {'L', 'S', 'R'};
Robot robot_controller{
    mov,
    line_sensors,
    color_sensor,
    claw,
    route,
    sizeof(route) / sizeof(route[0])};

void setup()
{
    Serial.begin(9600);
    robot_controller.setup();
    robot_controller.start();
}

void loop()
{
}
