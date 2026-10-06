#include <RobotComponents.h>

void read_rgb_sensor(RgbColor &color)
{
    // Reemplazar con la lectura del sensor RGB.
    color = {0, 0, 0};
}

Movement mov{2, 3, 4, 5};
LineSensors<1> line_sensors{(const int []){6}};
ColorSensor color_sensor{read_rgb_sensor, {255, 0, 0}, 30};
Claw claw{3, 20, 90};

const char route[] = {'L', 'S', 'R'};
Robot<1> robot_controller{
    mov,
    line_sensors,
    color_sensor,
    claw,
    route,
    3};

void setup()
{   
    Serial.begin(9600);
    robot_controller.setup();
    robot_controller.start();
}

void loop(){}
