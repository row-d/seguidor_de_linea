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

    void beep(int duration_ms)
    {
        digitalWrite(pin, HIGH);
        delay(duration_ms);
        digitalWrite(pin, LOW);
    }
};
#endif