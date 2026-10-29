#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H
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

    long read_distance()
    {
        digitalWrite(trigger_pin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigger_pin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigger_pin, LOW);

        long duration = pulseIn(echo_pin, HIGH, 25000UL);
        if (duration == 0)
            return 999;
        return duration / 29 / 2; // Convert to cm
    }

private:
    int trigger_pin;
    int echo_pin;
};

#endif