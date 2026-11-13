#ifndef SPEED_SETTINGS_TASK_H
#define SPEED_SETTINGS_TASK_H

#include <EEPROM.h>
#include "./robot.h"

const uint16_t SPEED_SETTINGS_MAGIC = 0x5653;
// La calibracion de sensores empieza en 0 y las ganancias del PID en 32 (14 bytes).
const int SPEED_SETTINGS_EEPROM_ADDRESS = 48;
// Giro tipo tanque: aparte de las velocidades para no invalidar las ya guardadas.
const uint16_t TANK_TURN_MAGIC = 0x544B;
const int TANK_TURN_EEPROM_ADDRESS = 60;

struct TankTurnRecord
{
    uint16_t magic;
    bool enabled;
};

struct SpeedSettingsRecord
{
    uint16_t magic;
    SpeedSettings speeds;
};

// Carga las velocidades guardadas. Sin datos validos quedan las de robot.h.
// Hay que llamarla en setup(), antes de que arranque FreeRTOS.
template <size_t N>
void Robot<N>::load_speeds()
{
    SpeedSettingsRecord record;
    EEPROM.get(SPEED_SETTINGS_EEPROM_ADDRESS, record);
    if (record.magic == SPEED_SETTINGS_MAGIC)
        speed_settings = record.speeds;

    movement.set_min_speed(speed_settings.min);

    TankTurnRecord tank;
    EEPROM.get(TANK_TURN_EEPROM_ADDRESS, tank);
    if (tank.magic == TANK_TURN_MAGIC)
        tank_turn = tank.enabled;
}

// Activa o desactiva el giro tipo tanque (ver tank_turn en robot.h) y lo guarda en
// EEPROM. Vale desde el siguiente ciclo de control. Devuelve el estado nuevo.
template <size_t N>
bool Robot<N>::toggle_tank_turn()
{
    tank_turn = !tank_turn;
    const TankTurnRecord tank = {TANK_TURN_MAGIC, tank_turn};
    EEPROM.put(TANK_TURN_EEPROM_ADDRESS, tank);
    return tank_turn;
}

// Cambia las velocidades y las guarda en EEPROM. Valen desde el siguiente ciclo de
// control, sin detener el recorrido. La llama la tarea de telemetria.
// Devuelve false, sin cambiar nada, si algun valor esta fuera de 0..255.
template <size_t N>
bool Robot<N>::set_speeds(int base, int turn, int min)
{
    const int values[] = {base, turn, min};
    for (int value : values)
    {
        if (value < 0 || value > 255)
            return false;
    }

    // Un int ocupa 2 bytes: sin la seccion critica, la tarea de control podria leerlo
    // a medio escribir.
    taskENTER_CRITICAL();
    speed_settings.base = base;
    speed_settings.turn = turn;
    speed_settings.min = min;
    movement.set_min_speed(min);
    taskEXIT_CRITICAL();

    const SpeedSettingsRecord record = {SPEED_SETTINGS_MAGIC, {base, turn, min}};
    EEPROM.put(SPEED_SETTINGS_EEPROM_ADDRESS, record);
    return true;
}

// Formato CSV: speeds,<base>,<giro>,<minima> y tank,on o tank,off. Solo la tarea de
// telemetria escribe en el puerto.
template <size_t N>
void Robot<N>::log_speeds()
{
    logger.print("speeds,");
    logger.print(speed_settings.base);
    logger.print(',');
    logger.print(speed_settings.turn);
    logger.print(',');
    logger.println(speed_settings.min);
    logger.println(tank_turn ? "tank,on" : "tank,off");
}

#endif
