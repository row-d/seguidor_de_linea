#ifndef LINE_SENSORS_H
#define LINE_SENSORS_H
#include "component.h"

template <size_t N>
class LineSensors : public Component
{
public:
    enum State
    {
        NONE,
        LEFT,
        RIGHT,
        BOTH
    };

private:
    const int (&sensor_pins)[N];

    bool detects_line(pin sensor_pin)
    {
        return digitalRead(sensor_pin);
    }

public:
    LineSensors(const int (&pins)[N])
        : sensor_pins(pins) {}

    void setup()
    {
        for (size_t i = 0; i < N; ++i)
        {
            pinMode(sensor_pins[i], INPUT_PULLUP);
        }
    }

    bool left_detected()
    {
        const size_t half_size = N / 2;
        for (size_t i = 0; i < half_size; ++i)
        {
            if (detects_line(sensor_pins[i]))
                return true;
        }
        return false;
    }

    bool right_detected()
    {
        const size_t half_size = N / 2;
        for (size_t i = half_size; i < N; ++i)
        {
            if (detects_line(sensor_pins[i]))
                return true;
        }
        return false;
    }

    bool line_detected()
    {
        return left_detected() || right_detected();
    }

    int raw_read(size_t sensor_index = 0)
    {
        return digitalRead(sensor_pins[sensor_index]);
    }

    bool intersection_detected()
    {
        return left_detected() && right_detected();
    }

    State read()
    {
        const bool left = left_detected();
        const bool right = right_detected();

        if (left && right)
            return BOTH;
        if (left)
            return LEFT;
        if (right)
            return RIGHT;
        return NONE;
    }
};

#endif
