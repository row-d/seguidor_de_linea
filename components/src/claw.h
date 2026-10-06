#ifndef CLAW_H
#define CLAW_H

#include <Servo.h>
#include "component.h"

class Claw : public Component
{
private:
    Servo servo;
    pin servo_pin;
    int open_angle;
    int closed_angle;

public:
    Claw(pin control_pin, int open_position, int closed_position)
        : servo_pin(control_pin),
          open_angle(open_position),
          closed_angle(closed_position) {}

    void setup()
    {
        servo.attach(servo_pin);
        close();
    }

    void open()
    {
        servo.write(open_angle);
    }

    void close()
    {
        servo.write(closed_angle);
    }
};

#endif
