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

    void turn_only(size_t index)
    {
        turn_off_all();
        set_led(index, true);
    }

    // LED 0 izquierdo y 1 derecho: enciende el del lado hacia el que gira, o los dos si
    // la diferencia entre ruedas no pasa de straight_deadband. Gira hacia la rueda lenta.
    void show_turn(int speed_l, int speed_r, int straight_deadband)
    {
        const int diff = speed_r - speed_l;
        set_led(0, diff >= -straight_deadband);
        set_led(1, diff <= straight_deadband);
    }

    void turn_off_all()
    {
        for (size_t i = 0; i < N; ++i)
            set_led(i, false);
    }

private:
    const int (&pins)[N];
};

#endif