#ifndef DEBUG_TASK_H
#define DEBUG_TASK_H
#include "robot.h"

template <size_t N>
void Robot<N>::report_line()
{
    // CSV: ms,raw0..rawN-1,error,found,state,correction,left_pwm,right_pwm.
    Serial.print(F("ms"));
    for (size_t i = 0; i < N; ++i)
    {
        Serial.print(F(",raw"));
        Serial.print(i);
    }
    Serial.println(F(",error,found,state,correction,left_pwm,right_pwm"));
    TickType_t wake = xTaskGetTickCount();
    while (true)
    {
        Telemetry report;
        if (xQueuePeek(telemetry_queue, &report, 0) == pdTRUE)
        {
            Serial.print(millis());
            for (size_t i = 0; i < N; ++i)
            {
                Serial.print(',');
                Serial.print(report.sensors.raw[i]);
            }
            Serial.print(','); Serial.print(report.sensors.error);
            Serial.print(','); Serial.print(report.sensors.found ? 1 : 0);
            Serial.print(','); Serial.print(static_cast<int>(report.state));
            Serial.print(','); Serial.print(report.correction);
            Serial.print(','); Serial.print(report.left_speed);
            Serial.print(','); Serial.println(report.right_speed);
        }
        // Low priority, limited output; serial transmission never blocks control.
        vTaskDelayUntil(&wake, ticks_for(200));
    }
}

template <size_t N>
void Robot<N>::debug()
{
    // Diagnostic mode is selected in start(config); never command the motors here.
}
#endif
