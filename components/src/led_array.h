#ifndef LED_ARRAY_H
#define LED_ARRAY_H

#include "component.h"

template <size_t N>
class LedArray : public Component
{
public:
    LedArray(const int (&pins)[N]) : pins(pins) {}

    void setup() override
    {
        for (size_t i = 0; i < N; ++i)
        {
            pinMode(pins[i], OUTPUT);
        }
    }

    void set_led(size_t index, bool state)
    {
        if (index < N)
        {
            digitalWrite(pins[index], state ? HIGH : LOW);
        }
    }

private:
    const int (&pins)[N];
};

#endif