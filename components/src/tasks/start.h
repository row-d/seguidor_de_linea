#ifndef START_TASK_H
#define START_TASK_H
#include "robot.h"
#include <math.h>

template <size_t N>
bool Robot<N>::start(const LineControlConfig &config)
{
    if (started) return true;
    if (config.base_speed <= 0 || config.base_speed > 255 ||
        config.max_correction <= 0 || config.max_correction > config.base_speed ||
        config.max_correction > 255 - config.base_speed ||
        config.search_speed <= 0 || config.search_speed > 255 ||
        config.period_ms == 0 || config.period_ms > 1000 ||
        config.sensor_timeout_ms < 2UL * config.period_ms ||
        config.search_switch_ms == 0 || config.search_timeout_ms < config.search_switch_ms ||
        !isfinite(config.kp) || !isfinite(config.ki) || !isfinite(config.kd) ||
        config.kp < 0 || config.ki < 0 || config.kd < 0)
    {
        movement.stop();
        return false;
    }
    line_config = config;
    // Prevent partially created tasks from running if start() is called at runtime.
    vTaskSuspendAll();
    sensor_queue = xQueueCreate(1, sizeof(LineSample));
    if (config.diagnostics)
        telemetry_queue = xQueueCreate(1, sizeof(Telemetry));
    bool ok = sensor_queue && (!config.diagnostics || telemetry_queue);
    if (ok)
        ok = xTaskCreate([](void *p) { static_cast<Robot<N> *>(p)->acquire_line(); },
                         "LineSensors", 256, this, 3, &sensor_task) == pdPASS;
    if (ok)
        ok = xTaskCreate([](void *p) { static_cast<Robot<N> *>(p)->follow_line(); },
                         "LineControl", 384, this, 2, &control_task) == pdPASS;
    if (ok && config.diagnostics)
        ok = xTaskCreate([](void *p) { static_cast<Robot<N> *>(p)->report_line(); },
                         "LineReport", 256, this, 1, &diagnostic_task) == pdPASS;
    if (!ok)
    {
        if (sensor_task) vTaskDelete(sensor_task);
        if (control_task) vTaskDelete(control_task);
        if (diagnostic_task) vTaskDelete(diagnostic_task);
        if (sensor_queue) vQueueDelete(sensor_queue);
        if (telemetry_queue) vQueueDelete(telemetry_queue);
        sensor_task = control_task = diagnostic_task = nullptr;
        sensor_queue = telemetry_queue = nullptr;
        movement.stop();
    }
    started = ok;
    xTaskResumeAll();
    return ok;
}
#endif
