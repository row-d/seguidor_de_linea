#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include "component.h"
#include "movement.h"
#include "line_sensors.h"
#include "color_sensor.h"
#include "claw.h"

template <size_t N>
class Robot : public Component
{
public:
    enum State
    {
        FOLLOW_LINE,
        HANDLE_INTERSECTION,
        DROP_OBJECT,
        FINISHED
    };

private:
    enum EventType
    {
        LINE_STATE_EVENT,
        OBJECT_DETECTED_EVENT
    };

    struct Event
    {
        EventType type;
        typename LineSensors<N>::State line_state;
    };

    Movement &movement;
    LineSensors<N> &line_sensors;
    ColorSensor &color_sensor;
    Claw &claw;
    const char *route;
    uint8_t route_length;
    uint8_t route_index;
    State state;
    typename LineSensors<N>::State last_line_state;
    QueueHandle_t events;

public:
    Robot(Movement &movement_controller,
          LineSensors<N> &sensors,
          ColorSensor &color,
          Claw &robot_claw,
          const char *route_definition,
          uint8_t number_of_intersections)
        : movement(movement_controller),
          line_sensors(sensors),
          color_sensor(color),
          claw(robot_claw),
          route(route_definition),
          route_length(number_of_intersections),
          route_index(0),
          state(FOLLOW_LINE),
          last_line_state(LineSensors<N>::NONE),
          events(nullptr) {}

    void setup()
    {
        movement.setup();
        line_sensors.setup();
        // color_sensor.setup();
        // claw.setup();
        // events = xQueueCreate(8, sizeof(Event));
    }

    void start()
    {
        xTaskCreate([](void *parameter)
                    {
                        Robot<N> *robot = static_cast<Robot<N> *>(parameter);
                        while (1)
                        {
                            robot->movement.move_forward(300);
                            vTaskDelay(pdMS_TO_TICKS(1000));
                            robot->movement.move_left(300);
                            vTaskDelay(pdMS_TO_TICKS(1000));
                            robot->movement.move_right(300);
                            vTaskDelay(pdMS_TO_TICKS(1000));
                            robot->movement.move_backward(300);
                            vTaskDelay(pdMS_TO_TICKS(1000));
                        } },
                    "Test", 192, this, 1, nullptr);
    }
};

#endif
