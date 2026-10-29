#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include "../components/component.h"
#include "../components/movement.h"
#include "../components/line_sensors.h"
#include "../components/color_sensor.h"
#include "../components/claw.h"
#include "../components/buzzer.h"
#include "../components/led_array.h"
#include "../components/ultrasonic_sensor.h"

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

public:
    Robot(Movement &movement_controller,
          LineSensors<N> &sensors,
          ColorSensor &color,
          Claw &robot_claw,
          Buzzer &robot_buzzer,
          LedArray<2> &robot_turn_leds,
          UltrasonicSensor &robot_obstacle_sensor)
        : movement(movement_controller),
          line_sensors(sensors),
          color_sensor(color),
          claw(robot_claw),
          buzzer(robot_buzzer),
          leds(robot_turn_leds),
          obstacle_sensor(robot_obstacle_sensor) {}

    void setup()
    {
        movement.setup();
        line_sensors.setup();
        color_sensor.setup();
        claw.setup();
        buzzer.setup();
        leds.setup();
        obstacle_sensor.setup();
    }

    // TASKS
    void start();
    void collision();
    void debug();
};

#endif
