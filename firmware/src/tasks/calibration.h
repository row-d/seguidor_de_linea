#ifndef CALIBRATION_TASK_H
#define CALIBRATION_TASK_H

#include <EEPROM.h>
#include "./robot.h"

const uint16_t SENSOR_CALIBRATION_MAGIC = 0x4C53;

template <size_t N>
struct SensorCalibration
{
    uint16_t magic;
    int thresholds[N];
};

template <size_t N>
void Robot<N>::calibrate()
{
    xTaskCreate([](void *parameter)
                {
                    Robot<N> *robot = static_cast<Robot<N> *>(parameter);
                    const unsigned long calibration_duration_ms = 10000;
                    const int sample_period_ms = 25;
                    int minimums[N];
                    int maximums[N];
                    SensorCalibration<N> calibration;

                    robot->movement.stop();
                    for (size_t index = 0; index < N; ++index)
                    {
                        minimums[index] = 1023;
                        maximums[index] = 0;
                    }

                    const unsigned long start_time = millis();
                    while (millis() - start_time < calibration_duration_ms)
                    {
                        for (size_t index = 0; index < N; ++index)
                        {
                            const int reading = robot->line_sensors.raw_read(index);
                            minimums[index] = min(minimums[index], reading);
                            maximums[index] = max(maximums[index], reading);
                        }

                        vTaskDelay(pdMS_TO_TICKS(sample_period_ms));
                    }

                    calibration.magic = SENSOR_CALIBRATION_MAGIC;
                    for (size_t index = 0; index < N; ++index)
                    {
                        const int range = maximums[index] - minimums[index];
                        // Un rango pequeno es solo ruido: el sensor no paso por la linea.
                        // Un tercio del rango y no la mitad: el maximo se toma con el sensor
                        // justo encima de la linea, pero al seguirla la lectura baja bastante.
                        const int threshold = range >= 100
                                                  ? minimums[index] + range / 3
                                                  : robot->line_sensors.threshold(index);
                        calibration.thresholds[index] = threshold;
                        robot->line_sensors.set_threshold(index, threshold);
                    }
                    EEPROM.put(0, calibration);

                    robot->logger.println("calibration,sensor,min,max,threshold");
                    for (size_t index = 0; index < N; ++index)
                    {
                        robot->logger.print("calibration,");
                        robot->logger.print(index);
                        robot->logger.print(',');
                        robot->logger.print(minimums[index]);
                        robot->logger.print(',');
                        robot->logger.print(maximums[index]);
                        robot->logger.print(',');
                        robot->logger.println(calibration.thresholds[index]);
                    }
                    robot->logger.println("calibration,complete");
                    vTaskDelete(nullptr); },
                "Calibration", 384, this, 1, nullptr);
}

template <size_t N>
void Robot<N>::load_calibration()
{
    SensorCalibration<N> calibration;
    EEPROM.get(0, calibration);
    if (calibration.magic != SENSOR_CALIBRATION_MAGIC)
        return;
    Serial.println("calibration,loaded");
    Serial.print("EEPROM thresholds: ");
    for (size_t index = 0; index < N; ++index)
    {
        line_sensors.set_threshold(index, calibration.thresholds[index]);
        Serial.print(calibration.thresholds[index]);
        if (index < N - 1)
            Serial.print(", ");
    }
    Serial.println();
}

#endif
