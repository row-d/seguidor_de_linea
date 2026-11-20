#ifndef BUZZER_H
#define BUZZER_H
#include <Arduino_FreeRTOS.h>
#include "component.h"

class Buzzer : public Component
{
private:
    int pin;

public:
    Buzzer(int pin) : pin(pin) {}

    void setup() override
    {
        pinMode(pin, OUTPUT);
    }

    void beep(int duration_ms, unsigned int frequency = 2000)
    {
        tone(pin, frequency, duration_ms);
        delay(duration_ms);
        noTone(pin);
    }

    // Solo desde una tarea: espera con vTaskDelay.
    void beep_repeat(uint8_t times, int duration_ms, int pause_ms)
    {
        for (uint8_t i = 0; i < times; ++i)
        {
            beep(duration_ms);
            vTaskDelay(pdMS_TO_TICKS(pause_ms));
        }
    }
};
#endif
