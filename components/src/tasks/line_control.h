#ifndef LINE_CONTROL_H
#define LINE_CONTROL_H
#include <PID_v1.h>
#include "robot.h"
#include "line_recovery.h"

template <size_t N>
void Robot<N>::follow_line()
{
    double input = 0, output = 0, target = 0;
    PID pid(&input, &output, &target, 0.0, 0.0, 0.0, REVERSE);
    pid.SetTunings(line_config.kp, line_config.ki, line_config.kd);
    pid.SetOutputLimits(-line_config.max_correction, line_config.max_correction);
    pid.SetSampleTime(line_config.period_ms);
    pid.SetMode(AUTOMATIC);
    pid.SetMode(MANUAL);
    LineRecovery recovery;
    bool tracking = false;
    const TickType_t timeout = ticks_for(line_config.sensor_timeout_ms);
    while (true)
    {
        Telemetry report = {};
        const bool received = xQueueReceive(sensor_queue, &report.sensors, timeout) == pdTRUE;
        const unsigned long now = millis();
        if (!received || !report.sensors.valid ||
            now - report.sensors.captured_ms > line_config.sensor_timeout_ms)
        {
            report.state = SENSOR_FAULT;
            recovery.reset();
        }
        else if (line_config.calibration_mode)
        {
            report.state = CALIBRATING;
            recovery.reset();
        }
        else if (report.sensors.ambiguous)
        {
            // 101 and 111 cannot distinguish an intersection from a finish mark.
            report.state = AMBIGUOUS;
            recovery.reset();
        }
        else if (report.sensors.found)
        {
            report.state = TRACKING;
            input = report.sensors.error;
            recovery.observe(report.sensors.error);
            if (!tracking)
            {
                // Initialize derivative history from the newly acquired line.
                pid.SetMode(MANUAL);
                output = 0;
                pid.SetMode(AUTOMATIC);
            }
            pid.Compute();
            report.correction = constrain(static_cast<int>(output),
                                          -line_config.max_correction, line_config.max_correction);
            report.left_speed = line_config.base_speed + report.correction;
            report.right_speed = line_config.base_speed - report.correction;
        }
        else
        {
            const int direction = recovery.direction(now, line_config.search_switch_ms,
                                                     line_config.search_timeout_ms);
            report.state = direction == 0 ? STOPPED : SEARCHING;
            report.left_speed = direction * line_config.search_speed;
            report.right_speed = -report.left_speed;
        }

        tracking = report.state == TRACKING;
        if (!tracking) pid.SetMode(MANUAL);
        if (report.state == TRACKING)
        {
            movement.move_forward(report.left_speed, report.right_speed);
            if (report.correction < -8) leds.turn_only(0);
            else if (report.correction > 8) leds.turn_only(1);
            else leds.turn_off_all();
        }
        else
        {
            leds.turn_off_all();
            if (report.state == SEARCHING && report.left_speed < 0)
                movement.rotate_left(line_config.search_speed);
            else if (report.state == SEARCHING)
                movement.rotate_right(line_config.search_speed);
            else
                movement.stop();
        }
        if (telemetry_queue) xQueueOverwrite(telemetry_queue, &report);
    }
}
#endif
