#ifndef DEBUG_TASK_H
#define DEBUG_TASK_H

#include "./robot.h"

template <size_t N>
void Robot<N>::debug()
{
    xTaskCreate([](void *parameter)
                {
                    Robot<N> *robot = static_cast<Robot<N> *>(parameter);

                    // Serial.print("Line Sensor Readings: ");
                    // auto s1 = robot->line_sensors.raw_read(0);
                    // auto s2 = robot->line_sensors.raw_read(1);
                    // auto s3 = robot->line_sensors.raw_read(2);
                    // Serial.print(s1);
                    // Serial.print(", ");
                    // Serial.print(s2);
                    // Serial.print(", ");
                    // Serial.println(s3);
                    // vTaskDelay(pdMS_TO_TICKS(1000));

                    // Serial.print("Leds:");
                    // for (size_t i = 0; i < 2; ++i)
                    // {
                    //     robot->leds.turn_only(i);
                    // }
                    // Serial.println();
                    // vTaskDelay(pdMS_TO_TICKS(1000));

                    // Serial.print("Buzzer: ON");
                    // robot->buzzer.beep(1000);
                    // vTaskDelay(pdMS_TO_TICKS(1000));

                    // Motores
                    
                    robot->movement.move_forward(100);
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    robot->movement.move_backward(100);
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    robot->movement.rotate_left(185);
                    vTaskDelay(pdMS_TO_TICKS(2000));
                    robot->movement.rotate_right(185);
                    vTaskDelay(pdMS_TO_TICKS(2000));
                    robot->movement.stop();

                    vTaskDelete(nullptr);
                },
                "LineDebug", 192, this, 1, nullptr);
}

#endif