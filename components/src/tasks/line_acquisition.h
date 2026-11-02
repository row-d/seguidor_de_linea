#ifndef LINE_ACQUISITION_H
#define LINE_ACQUISITION_H
#include "robot.h"

template <size_t N>
void Robot<N>::acquire_line()
{
    const TickType_t period = ticks_for(line_config.period_ms);
    TickType_t wake = xTaskGetTickCount();
    while (true)
    {
        const LineSample value = line_sensors.sample();
        // One-element mailbox: the controller always receives the newest sample.
        xQueueOverwrite(sensor_queue, &value);
        vTaskDelayUntil(&wake, period);
    }
}
#endif
