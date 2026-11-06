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
#include "../components/logger.h"
#include "route_memory.h"

const int BASE_SPEED = 130;
const int TURN_SPEED = 185;

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

    void wait_ms(uint32_t duration_ms)
    {
        vTaskDelay(pdMS_TO_TICKS(duration_ms));
    }

    CrossResult cross_horizontal_line();
    void show_route(Route route);
    void log_header();
    void turn_toward(Route side, uint32_t min_turn_ms);
    void halt(const char *reason);
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
    }

    // TASKS
    void start(Route route = ROUTE_LEFT);
    void calibrate();
    void load_calibration();
    void debug();
};

#endif
