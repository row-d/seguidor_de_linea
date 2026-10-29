#ifndef MOVEMENT_H
#define MOVEMENT_H

#include "component.h"

class Movement : public Component
{
private:
    pin enable_l;
    pin direction_l;
    pin enable_r;
    pin direction_r;
    int max_speed = 255;

public:
    Movement(pin _enable_l, pin _direction_l,
             pin _enable_r, pin _direction_r)
        : enable_l(_enable_l), direction_l(_direction_l),
          enable_r(_enable_r), direction_r(_direction_r)
    {
    }

    void setup();
    void update_speed(int speed_l, int speed_r);
    void update_speed(int speed);
    void move_forward(int speed);
    void move_forward(int speed_l, int speed_r);
    void move_backward(int speed);
    void move_left(int speed, int direction = HIGH);
    void move_right(int speed, int direction = HIGH);
    void rotate_left(int speed);
    void rotate_right(int speed);
    void stop();
};

void Movement::setup()
{
    pinMode(enable_l, OUTPUT);
    pinMode(direction_l, OUTPUT);
    pinMode(enable_r, OUTPUT);
    pinMode(direction_r, OUTPUT);

    digitalWrite(direction_l, LOW);
    digitalWrite(direction_r, HIGH);
    stop();
}

void Movement::update_speed(int speed_l, int speed_r)
{
    analogWrite(enable_l, constrain(speed_l, 0, this->max_speed));
    analogWrite(enable_r, constrain(speed_r, 0, this->max_speed));
}

void Movement::update_speed(int speed)
{
    update_speed(speed, speed);
}

void Movement::move_forward(int speed)
{
    digitalWrite(direction_l, LOW);
    digitalWrite(direction_r, HIGH);
    update_speed(speed, speed);
}

void Movement::move_forward(int speed_l, int speed_r)
{
    digitalWrite(direction_l, LOW);
    digitalWrite(direction_r, HIGH);
    update_speed(speed_l, speed_r);
}

void Movement::move_backward(int speed)
{
    digitalWrite(direction_l, HIGH);
    digitalWrite(direction_r, LOW);
    update_speed(speed, speed);
}

void Movement::move_left(int speed, int direction = HIGH)
{

    digitalWrite(direction_l, direction == HIGH ? LOW : HIGH);
    digitalWrite(direction_r, direction == HIGH ? HIGH : LOW);
    update_speed(speed, 0);
}

void Movement::move_right(int speed, int direction = HIGH)
{
    digitalWrite(direction_l, direction == HIGH ? LOW : HIGH);
    digitalWrite(direction_r, direction == HIGH ? HIGH : LOW);
    update_speed(0, speed);
}

void Movement::rotate_left(int speed)
{
    digitalWrite(direction_l, HIGH);
    digitalWrite(direction_r, HIGH);
    update_speed(speed, speed);
}

void Movement::rotate_right(int speed)
{
    digitalWrite(direction_l, LOW);
    digitalWrite(direction_r, LOW);
    update_speed(speed, speed);
}

void Movement::stop()
{
    update_speed(0);
}

#endif // MOVEMENT_H
