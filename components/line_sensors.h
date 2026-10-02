#ifndef LINE_SENSORS_H
#define LINE_SENSORS_H
#include "component.h"
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
    const int *sensor_pins;
    const size_t num_sensors;

    bool detects_line(pin sensor_pin)
    {
        return digitalRead(sensor_pin);
    }

public:
    LineSensors(const int *pins,
                size_t num_pins)
        : sensor_pins(pins),
          num_sensors(num_pins) {}

    void setup()
    {
        for (size_t i = 0; i < num_sensors; ++i)
        {
            pinMode(sensor_pins[i], INPUT_PULLUP);
        }
    }

    bool left_detected()
    {
        if (num_sensors == 1)
            return detects_line(sensor_pins[0]);
        const size_t half_size = num_sensors / 2;
        for (size_t i = 0; i < half_size; ++i)
        {
            if (detects_line(sensor_pins[i]))
                return true;
        }
        return false;
    }

    bool right_detected()
    {
        if (num_sensors == 1)
            return detects_line(sensor_pins[0]);
        const size_t half_size = num_sensors / 2;
        for (size_t i = half_size; i < num_sensors; ++i)
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
