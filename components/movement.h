#ifndef MOVEMENT_H
#define MOVEMENT_H

#include "component.h"

typedef int pin;

class Movement : public Component
{
private:
    pin trigger_l;
    pin speed_l;
    pin trigger_r;
    pin speed_r;

    void set_rotation(int direction, int speed);

public:
    Movement(pin _trigger_pin_l, pin _speed_pin_l, pin _trigger_pin_r, pin _speed_pin_r)
        : trigger_l(_trigger_pin_l), speed_l(_speed_pin_l),
          trigger_r(_trigger_pin_r), speed_r(_speed_pin_r)
    {
    }

    void setup();
    void update_speed(int speed_l, int speed_r);
    void update_speed(int speed);
    void move_forward(int speed);
    void move_forward(int left_speed, int right_speed);
    void move_backward(int speed);
    void stop();
    void rotate_timed(float degrees, int speed, unsigned long time_for_90_degrees_ms);
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
    analogWrite(speed_l, speed_wheel_tl);
    analogWrite(speed_r, speed_wheel_tr);
}

void Movement::update_speed(int speed)
{
    update_speed(speed, speed);
}

void Movement::move_forward(int speed)
{
    digitalWrite(trigger_l, HIGH);
    digitalWrite(trigger_r, HIGH);
    update_speed(speed);
}

void Movement::move_forward(int left_speed, int right_speed)
{
    digitalWrite(trigger_l, HIGH);
    digitalWrite(trigger_r, HIGH);
    update_speed(left_speed, right_speed);
}

void Movement::move_backward(int speed)
{
    digitalWrite(trigger_l, HIGH);
    digitalWrite(trigger_r, HIGH);
    update_speed(speed);
}

void Movement::stop()
{
    update_speed(0);
}

void Movement::rotate_timed(float degrees, int speed,
                            unsigned long time_for_90_degrees_ms)
{
    if (degrees == 0.0f || speed <= 0 || time_for_90_degrees_ms == 0)
    {
        stop();
        return;
    }

    const float duration_ms = time_for_90_degrees_ms * abs(degrees) / 90.0f;
    set_rotation(degrees > 0.0f ? 1 : -1, speed);
    delay((unsigned long)duration_ms);
    stop();
}

void Movement::set_rotation(int direction, int speed)
{
    digitalWrite(trigger_l, direction > 0 ? LOW : HIGH);
    digitalWrite(trigger_r, direction > 0 ? HIGH : LOW);
    update_speed(speed);
}

#endif // MOVEMENT_H
