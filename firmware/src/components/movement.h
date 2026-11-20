#ifndef MOVEMENT_H
#define MOVEMENT_H

#include "component.h"

class Movement : public Component
{
public:
    static const int MAX_SPEED = 255;

private:
    // Cada rueda: pines del driver y compensacion de drive_compensated().
    struct Wheel
    {
        pin enable;
        pin direction;
        uint8_t forward; // nivel de direction para avanzar
        int min_pwm;
        float trim;

        void setup()
        {
            pinMode(enable, OUTPUT);
            pinMode(direction, OUTPUT);
        }

        void drive(int pwm)
        {
            digitalWrite(direction, pwm >= 0 ? forward : !forward);
            analogWrite(enable, abs(pwm));
        }

        // Salta la zona muerta del motor: cualquier velocidad distinta de 0 da al menos
        // min_pwm, asi la rueda interior de una curva no se queda parada con un PWM que no
        // la mueve. trim escala la parte por encima del minimo.
        int to_pwm(int speed) const
        {
            speed = constrain(speed, -MAX_SPEED, MAX_SPEED);
            if (speed == 0)
                return 0;
            const long above_min = (long)(abs(speed) - 1) * (MAX_SPEED - min_pwm) / (MAX_SPEED - 1);
            const int magnitude = min(min_pwm + (int)(above_min * trim), MAX_SPEED);
            return speed > 0 ? magnitude : -magnitude;
        }
    };

    Wheel left;
    Wheel right;

public:
    Movement(pin enable_l, pin direction_l, pin enable_r, pin direction_r)
        : left{enable_l, direction_l, LOW, 0, 1.0f},
          right{enable_r, direction_r, HIGH, 0, 1.0f} {}

    void setup() override
    {
        left.setup();
        right.setup();
        drive(0, 0);
    }

    // PWM con signo de cada rueda, -255..255: negativo va hacia atras.
    void drive(int pwm_l, int pwm_r)
    {
        left.drive(pwm_l);
        right.drive(pwm_r);
    }

    // Deja de mover las ruedas sin cambiar su sentido de giro.
    void stop()
    {
        analogWrite(left.enable, 0);
        analogWrite(right.enable, 0);
    }

    // min_pwm: PWM minimo con el que arranca cada rueda. trim (<= 1): escala el PWM por
    // encima del minimo para que las dos ruedas corran igual con la misma velocidad.
    void set_compensation(int min_pwm_l, float trim_l, int min_pwm_r, float trim_r)
    {
        left.min_pwm = min_pwm_l;
        left.trim = trim_l;
        right.min_pwm = min_pwm_r;
        right.trim = trim_r;
    }

    // Como drive(), pero con velocidades -255..255 compensadas con set_compensation().
    void drive_compensated(int speed_l, int speed_r)
    {
        drive(left.to_pwm(speed_l), right.to_pwm(speed_r));
    }
};

#endif // MOVEMENT_H
