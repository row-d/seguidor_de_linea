#ifndef BUZZER_H
#define BUZZER_H
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
};
#endif