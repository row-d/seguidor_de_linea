#include <RobotComponents.h>

void read_rgb_sensor(RgbColor &color)
{
    // Reemplazar con la lectura del sensor RGB.
    color = {0, 0, 0};
}

Movement mov{2, 3, 4, 5};
<<<<<<< Updated upstream
LineSensors<3> line_sensors{(const int[]){0, 1, 2}};
=======
const int line_sensor_pins[] = {A0, A1, A2};
const int line_thresholds[] = {110, 120, 100};
const bool black_is_high = true; // Confirmar con lecturas sobre negro y fondo.
LineSensors<3> line_sensors{line_sensor_pins, line_thresholds, black_is_high};
// Activar tras medir los tres sensores. Los ceros son valores pendientes.
const bool use_line_calibration = false;
const int line_background[] = {0, 0, 0};
const int line_black[] = {0, 0, 0};
LineControlConfig line_control;
>>>>>>> Stashed changes
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
    // true: imprime lecturas para calibrar, con los motores detenidos.
    line_control.calibration_mode = false;
    if (use_line_calibration && !line_sensors.set_calibration(line_background, line_black))
    {
        Serial.println(F("ERROR: calibracion invalida; motores detenidos."));
        return;
    }
    if (!robot_controller.start(line_control))
        Serial.println(F("ERROR: no se pudo iniciar el control; motores detenidos."));
}

void loop() {}
