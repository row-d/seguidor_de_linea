#ifndef COLLISION_TASK_H
#define COLLISION_TASK_H

#include "robot.h"

template <size_t N>
void Robot<N>::collision()
{
    xTaskCreate([](void *parameter)
                {
        Robot<N> *robot = static_cast<Robot<N> *>(parameter);
        while(1){
            auto dist = robot->obstacle_sensor.read_distance();
            RgbColor color = robot->color_sensor.read();
            RgbColor white = {255, 255, 255};
            RgbColor black = {0, 0, 0};
            if (dist < 15 && !color.close_to(white, 30) && color.close_to(black, 30))
            {
                robot->movement.stop();
                robot->buzzer.beep(1000);
                robot->claw.open();
                vTaskDelay(pdMS_TO_TICKS(1000));
                robot->claw.close();
                vTaskDelay(pdMS_TO_TICKS(1000));

            }
        } },
                "collision", 192, this, 1, nullptr);
}

#endif