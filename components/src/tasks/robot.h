#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include "line_config.h"
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
private:
    using LineSample = typename LineSensors<N>::Sample;
    enum MotionState { STOPPED, TRACKING, SEARCHING, AMBIGUOUS, CALIBRATING, SENSOR_FAULT };
    struct Telemetry
    {
        LineSample sensors;
        MotionState state;
        int correction;
        int left_speed;
        int right_speed; // Signed only during in-place recovery.
    };
    LineControlConfig line_config;
    QueueHandle_t sensor_queue = nullptr;
    QueueHandle_t telemetry_queue = nullptr;
    TaskHandle_t sensor_task = nullptr;
    TaskHandle_t control_task = nullptr;
    TaskHandle_t diagnostic_task = nullptr;
    bool started = false;

    static TickType_t ticks_for(unsigned int milliseconds)
    {
        // Round up: on AVR a 30 ms request commonly needs two 16 ms ticks.
        return (milliseconds + portTICK_PERIOD_MS - 1UL) / portTICK_PERIOD_MS;
    }
    void acquire_line();
    void follow_line();
    void report_line();
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
    bool start(const LineControlConfig &config = LineControlConfig());
    void collision();
    void debug();
};

#endif
