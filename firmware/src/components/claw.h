#ifndef CLAW_H
#define CLAW_H

#include <Servo.h>
#include <Arduino_FreeRTOS.h>
#include "component.h"

// Garra con un servo que se mueve de step_degrees en step_degrees grados.
class Claw : public Component
{
private:
    Servo servo;
    pin servo_pin;
    int open_angle;
    int closed_angle;
    int step_degrees;
    TickType_t step_ticks;
    int position;

    // Solo desde una tarea: espera con vTaskDelay.
    void rotate_to(int to)
    {
        while (position != to)
        {
            // Avanza sin pasarse del destino, aunque el rango no sea multiplo del paso.
            position = position < to ? min(position + step_degrees, to) : max(position - step_degrees, to);
            servo.write(position);
            vTaskDelay(step_ticks);
        }
    }

public:
    Claw(pin control_pin, int open_position, int closed_position, int step_degrees = 1, int ms_per_step = 15)
        : servo_pin(control_pin),
          open_angle(open_position),
          closed_angle(closed_position),
          step_degrees(max(step_degrees, 1)),
          step_ticks(max(pdMS_TO_TICKS(ms_per_step), (TickType_t)1)),
          position(closed_position) {}

    // Se llama desde setup(), antes de arrancar FreeRTOS: no puede usar vTaskDelay,
    // asi que coloca la garra cerrada directamente.
    void setup() override
    {
        servo.write(position); // fija el pulso antes de attach para evitar un salto
        servo.attach(servo_pin);
    }

    void open()
    {
        rotate_to(open_angle);
    }

    void close()
    {
        rotate_to(closed_angle);
    }
};

#endif
