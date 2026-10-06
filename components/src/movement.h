#ifndef MOVEMENT_H
#define MOVEMENT_H

#include "component.h"

class Movement : public Component
{
private:
    pin trigger_l;
    pin speed_l;
    pin trigger_r;
    pin speed_r;

public:
    Movement(pin _trigger_pin_l, pin _speed_pin_l, pin _trigger_pin_r, pin _speed_pin_r)
        : trigger_l(_trigger_pin_l), speed_l(_speed_pin_l),
          trigger_r(_trigger_pin_r), speed_r(_speed_pin_r)
    {
    }

    void setup();
    void update_speed(int speed_l, int speed_r);
    void update_speed(int speed);
    void move(int left_speed, int right_speed);
    void move_forward(int speed);
    void move_backward(int speed);
    void move_left(int speed);
    void move_right(int speed);
};

void Movement::setup()
{
    pinMode(trigger_l, OUTPUT);
    pinMode(trigger_r, OUTPUT);
    pinMode(speed_l, OUTPUT);
    pinMode(speed_r, OUTPUT);
}

void Movement::update_speed(int speed_l, int speed_r)
{
    analogWrite(this->speed_l, speed_l);
    analogWrite(this->speed_r, speed_r);
}

void Movement::update_speed(int speed)
{
    update_speed(speed, speed);
}

void Movement::move(int left_speed, int right_speed)
{
    digitalWrite(trigger_l, HIGH);
    digitalWrite(trigger_r, HIGH);
    update_speed(left_speed, right_speed);
}

void Movement::move_forward(int speed)
{
    move(speed, -speed);
}

void Movement::move_backward(int speed)
{
    move(-speed, speed);
}

void Movement::move_left(int speed)
{
    move(-speed, -speed);
}

void Movement::move_right(int speed)
{
    move(speed, speed);
}

#endif // MOVEMENT_H
