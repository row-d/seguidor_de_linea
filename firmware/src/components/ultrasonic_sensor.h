#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H
#include <Arduino_FreeRTOS.h>
#include "component.h"

class UltrasonicSensor : public Component
{
public:
    UltrasonicSensor(int trigger_pin, int echo_pin) : trigger_pin(trigger_pin), echo_pin(echo_pin) {}

    void setup() override
    {
        pinMode(trigger_pin, OUTPUT);
        pinMode(echo_pin, INPUT);
    }

    // max_cm limita cuanto se espera el eco (unos 58 us por cm). Durante la espera el
    // planificador esta suspendido, asi que un alcance corto no frena a otras tareas.
    // Si no hay eco dentro del alcance devuelve 999.
    long read_distance(long max_cm = 430)
    {
        const unsigned long timeout_us = max_cm * 58UL + 1000UL; // + margen: el eco empieza ~0.5 ms despues del disparo
        // Se suspende el planificador para que otra tarea no corte la medicion, y se
        // usa pulseInLong (mide con micros()) porque pulseIn cuenta ciclos y da pulsos
        // mas cortos de lo real si entran interrupciones (Servo, Serial, tick).
        vTaskSuspendAll();
        digitalWrite(trigger_pin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigger_pin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigger_pin, LOW);

        long duration = pulseInLong(echo_pin, HIGH, timeout_us);
        xTaskResumeAll();
        if (duration == 0)
            return 999;
        return duration / 29 / 2; // Convert to cm
    }

private:
    int trigger_pin;
    int echo_pin;
};

#endif