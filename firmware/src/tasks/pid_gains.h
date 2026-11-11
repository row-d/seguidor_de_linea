#ifndef PID_GAINS_TASK_H
#define PID_GAINS_TASK_H

#include <EEPROM.h>
#include "./robot.h"

const uint16_t PID_GAINS_MAGIC = 0x5049;
// La calibracion de sensores ocupa desde la direccion 0 (2 + 2 * N bytes).
const int PID_GAINS_EEPROM_ADDRESS = 32;
// Limite de seguridad para Kp, Ki y Kd: un valor mayor es casi seguro un error al teclear.
const float PID_GAIN_MAX = 10.0;

struct PidGainsRecord
{
    uint16_t magic;
    float kp;
    float ki;
    float kd;
};

// Carga las ganancias guardadas. Sin datos validos quedan las de robot.h.
// Hay que llamarla en setup(), antes de que arranque FreeRTOS.
template <size_t N>
void Robot<N>::load_gains()
{
    PidGainsRecord record;
    EEPROM.get(PID_GAINS_EEPROM_ADDRESS, record);
    if (record.magic != PID_GAINS_MAGIC)
        return;

    pid_kp = record.kp;
    pid_ki = record.ki;
    pid_kd = record.kd;
}

// Cambia las ganancias, las guarda en EEPROM y avisa a la tarea de control, que las
// aplica al empezar su siguiente ciclo. La llama la tarea de telemetria.
// Devuelve false, sin cambiar nada, si algun valor esta fuera de rango.
template <size_t N>
bool Robot<N>::set_gains(float kp, float ki, float kd)
{
    const float values[] = {kp, ki, kd};
    for (float value : values)
    {
        // La primera comparacion tambien descarta NaN.
        if (!(value >= 0) || value > PID_GAIN_MAX)
            return false;
    }

    // Un float ocupa 4 bytes: sin la seccion critica, la tarea de control podria
    // leerlo a medio escribir.
    taskENTER_CRITICAL();
    pid_kp = kp;
    pid_ki = ki;
    pid_kd = kd;
    gains_changed = true;
    taskEXIT_CRITICAL();

    const PidGainsRecord record = {PID_GAINS_MAGIC, kp, ki, kd};
    EEPROM.put(PID_GAINS_EEPROM_ADDRESS, record);
    return true;
}

// Copia las ganancias actuales para la tarea de control y da el cambio por aplicado.
template <size_t N>
void Robot<N>::read_gains(float &kp, float &ki, float &kd)
{
    taskENTER_CRITICAL();
    kp = pid_kp;
    ki = pid_ki;
    kd = pid_kd;
    gains_changed = false;
    taskEXIT_CRITICAL();
}

// Formato CSV: pid,<kp>,<ki>,<kd>. Solo la tarea de telemetria escribe en el puerto.
template <size_t N>
void Robot<N>::log_gains()
{
    logger.print("pid,");
    logger.print_decimal(pid_kp);
    logger.print(',');
    logger.print_decimal(pid_ki);
    logger.print(',');
    logger.print_decimal(pid_kd);
    logger.println("");
}

#endif
