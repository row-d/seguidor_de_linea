#ifndef COLLISION_TASK_H
#define COLLISION_TASK_H

#include <Arduino_FreeRTOS.h>
#include "../components/ultrasonic_sensor.h"
#include "../components/claw.h"
#include "../components/logger.h"

// Con un objeto a menos de DISTANCE_CM en CONFIRMATIONS lecturas seguidas, abre y
// cierra la garra.
namespace collision
{
    const long DISTANCE_CM = 7;
    const int CONFIRMATIONS = 3;

    struct Components
    {
        UltrasonicSensor &sensor;
        Claw &claw;
        Logger &logger;
    };

    // parameters: Components.
    void task(void *parameters)
    {
        Components &hw = *static_cast<Components *>(parameters);
        int hits = 0;

        for (;;)
        {
            if (hw.sensor.read_distance(DISTANCE_CM * 2) <= DISTANCE_CM)
                hits++;
            else
                hits = 0;

            if (hits >= CONFIRMATIONS)
            {
                hw.logger.println("collision");
                hw.claw.open();
                hw.claw.close();
                hits = 0;
            }
            // >= 60 ms entre disparos para que el eco anterior no se lea como nuevo.
            vTaskDelay(pdMS_TO_TICKS(65));
        }
    }
}

#endif
