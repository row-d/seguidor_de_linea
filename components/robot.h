#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include "component.h"
#include "movement.h"
#include "line_sensors.h"
#include "color_sensor.h"
#include "claw.h"

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
        LineSensors::State line_state;
    };

    Movement &movement;
    LineSensors &line_sensors;
    ColorSensor &color_sensor;
    Claw &claw;
    const char *route;
    uint8_t route_length;
    uint8_t route_index;
    State state;
    LineSensors::State last_line_state;
    QueueHandle_t events;

    void read_sensors()
    {
        for (;;)
        {
            Event line_event{LINE_STATE_EVENT, line_sensors.read()};
            xQueueSend(events, &line_event, 0);

            if (color_sensor.object_detected())
            {
                Event object_event{OBJECT_DETECTED_EVENT, LineSensors::NONE};
                xQueueSend(events, &object_event, 0);
            }

            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }

    void control_robot()
    {
        Event event{};
        const int base_speed = 150;
        const int correction_speed = 70;
        const unsigned long time_for_90_degrees_ms = 1200;

        while (1)
        {
            while (xQueueReceive(events, &event, 0) == pdTRUE)
            {
                if (event.type == LINE_STATE_EVENT)
                {
                    last_line_state = event.line_state;
                }
                else if (event.type == OBJECT_DETECTED_EVENT && state == FOLLOW_LINE)
                {
                    state = DROP_OBJECT;
                }
            }

            if (state == FOLLOW_LINE)
            {
                if (last_line_state == LineSensors::BOTH)
                {
                    state = HANDLE_INTERSECTION;
                }
                else if (last_line_state == LineSensors::LEFT)
                {
                    movement.move_forward(correction_speed, base_speed);
                }
                else if (last_line_state == LineSensors::RIGHT)
                {
                    movement.move_forward(base_speed, correction_speed);
                }
                else
                {
                    movement.move_forward(base_speed, base_speed);
                }
            }
            else if (state == HANDLE_INTERSECTION)
            {
                handle_intersection(time_for_90_degrees_ms);
            }
            else if (state == DROP_OBJECT)
            {
                movement.stop();
                claw.open();
                vTaskDelay(pdMS_TO_TICKS(1000));
                state = FINISHED;
            }
            else
            {
                movement.stop();
            }

            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    void handle_intersection(unsigned long time_for_90_degrees_ms)
    {
        movement.stop();

        if (route_index >= route_length)
        {
            state = FINISHED;
            return;
        }

        const char turn = route[route_index++];
        if (turn == 'L')
            movement.rotate_timed(-90, 150, time_for_90_degrees_ms);

        else if (turn == 'R')
            movement.rotate_timed(90, 150, time_for_90_degrees_ms);

        else if (turn == 'S')
        {
            movement.move_forward(150);
            vTaskDelay(pdMS_TO_TICKS(250));
            movement.stop();
        }

        state = FOLLOW_LINE;
    }

public:
    Robot(Movement &movement_controller,
          LineSensors &sensors,
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
          last_line_state(LineSensors::NONE),
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
        // if (events == nullptr) return;

        // xTaskCreate([](void *parameter)
        //             { static_cast<Robot *>(parameter)->read_sensors(); }, "Sensors", 192, this, 2, nullptr);

        // xTaskCreate([](void *parameter)
        //             { static_cast<Robot *>(parameter)->control_robot(); }, "Control", 256, this, 2, nullptr);

        // xTaskCreate([](void *parameter)
        //             {
        //                 Robot *robot = static_cast<Robot *>(parameter);
        //                 while (1)
        //                 {
        //                     Serial.print("raw=");
        //                     Serial.println(robot->line_sensors.raw_read(0));
        //                 } },
        //             "Test", 192, this, 1, nullptr);

        xTaskCreate([](void *parameter)
                    {
                        Robot *robot = static_cast<Robot *>(parameter);
                        while (1)
                        {
                            robot->movement.move_forward(150);
                            vTaskDelay(pdMS_TO_TICKS(1000));
                        } 
                    },
                    "Test", 192, this, 1, nullptr);
    }
};

#endif
