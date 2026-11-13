#ifndef CALIBRATION_TASK_H
#define CALIBRATION_TASK_H

#include <EEPROM.h>
#include "./robot.h"

// Formato actual: lectura sobre el fondo y sobre la linea de cada sensor.
const uint16_t SENSOR_CALIBRATION_MAGIC = 0x4C54;
// Formato anterior: solo el umbral de cada sensor.
const uint16_t SENSOR_THRESHOLDS_MAGIC = 0x4C53;
// Rango minimo (linea - fondo) para dar por buena la medida de un sensor.
const int CALIBRATION_MIN_RANGE = 100;

template <size_t N>
struct SensorCalibration
{
    uint16_t magic;
    int white[N];
    int black[N];
};

template <size_t N>
struct SensorThresholds
{
    uint16_t magic;
    int thresholds[N];
};

// Calibracion manual: detiene los motores y mide durante 10 segundos. Hay que pasar a
// mano los sensores sobre el fondo y sobre la linea. (El robot no tiene fuerza para
// girar sobre su eje en el piso, asi que no puede barrer la linea por su cuenta.)
// La ejecuta la tarea "Calibration" (telemetry.h). No escribe en el puerto serie: deja
// el resultado en calibration_minimums y calibration_maximums, y la tarea de
// telemetria lo envia al ver CALIBRATION_DONE.
template <size_t N>
void Robot<N>::run_calibration()
{
    const unsigned long calibration_duration_ms = 10000;
    int *const minimums = calibration_minimums;
    int *const maximums = calibration_maximums;
    SensorCalibration<N> calibration;

    movement.stop();
    for (size_t index = 0; index < N; ++index)
    {
        minimums[index] = 1023;
        maximums[index] = 0;
    }
    calibration_state = CALIBRATION_RUNNING;

    const unsigned long start_time = millis();
    while (millis() - start_time < calibration_duration_ms)
    {
        for (size_t index = 0; index < N; ++index)
        {
            const int reading = line_sensors.raw_read(index);
            minimums[index] = min(minimums[index], reading);
            maximums[index] = max(maximums[index], reading);
        }

        // Un tick (unos 16 ms), para no perder la linea si se pasa rapido sobre ella.
        vTaskDelay(1);
    }

    calibration.magic = SENSOR_CALIBRATION_MAGIC;
    for (size_t index = 0; index < N; ++index)
    {
        // Un rango pequeno es solo ruido: el sensor no paso por la linea y conserva
        // sus niveles anteriores.
        if (maximums[index] - minimums[index] >= CALIBRATION_MIN_RANGE)
            line_sensors.set_levels(index, minimums[index], maximums[index]);
        calibration.white[index] = line_sensors.white_level(index);
        calibration.black[index] = line_sensors.black_level(index);
    }
    EEPROM.put(0, calibration);
    calibration_state = CALIBRATION_DONE;
}

template <size_t N>
void Robot<N>::load_calibration()
{
    SensorCalibration<N> calibration;
    EEPROM.get(0, calibration);
    if (calibration.magic == SENSOR_CALIBRATION_MAGIC)
    {
        for (size_t index = 0; index < N; ++index)
            line_sensors.set_levels(index, calibration.white[index], calibration.black[index]);
    }
    else if (calibration.magic == SENSOR_THRESHOLDS_MAGIC)
    {
        // Calibracion guardada con el formato anterior: conserva los umbrales y
        // estima los niveles hasta que se calibre de nuevo.
        SensorThresholds<N> previous;
        EEPROM.get(0, previous);
        for (size_t index = 0; index < N; ++index)
            line_sensors.set_threshold(index, previous.thresholds[index]);
    }
    else
    {
        return;
    }

    Serial.println("calibration,loaded");
    Serial.print("EEPROM thresholds: ");
    for (size_t index = 0; index < N; ++index)
    {
        Serial.print(line_sensors.threshold(index));
        if (index < N - 1)
            Serial.print(", ");
    }
    Serial.println();
}

#endif
