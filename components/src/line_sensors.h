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
        MIDDLE,
        INTERSECTION
    };

private:
    const int (&sensor_pins)[N];

    bool detects_line(pin sensor_pin)
    {
        return analogRead(sensor_pin);
    }

    bool is_middle_sensor(size_t sensor_index) const
    {
        const size_t middle = N / 2;
        return N % 2 == 0
                   ? sensor_index == middle - 1 || sensor_index == middle
                   : sensor_index == middle;
    }

    bool is_left_sensor(size_t sensor_index) const
    {
        return sensor_index < N / 2 && !is_middle_sensor(sensor_index);
    }

    bool is_right_sensor(size_t sensor_index) const
    {
        return sensor_index >= N / 2 && !is_middle_sensor(sensor_index);
    }

    bool side_detected(bool left_side)
    {
        for (size_t index = 0; index < N; ++index)
        {
            const bool belongs_to_side = left_side
                                             ? is_left_sensor(index)
                                             : is_right_sensor(index);
            if (belongs_to_side && detects_line(sensor_pins[index]))
                return true;
        }
        return false;
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
        return side_detected(true);
    }

    bool right_detected()
    {
        return side_detected(false);
    }

    bool middle_detected()
    {
        bool middle_found = false;
        for (size_t index = 0; index < N; ++index)
        {
            const bool detected = detects_line(sensor_pins[index]);
            if (is_middle_sensor(index))
                middle_found = middle_found || detected;
            else if (detected)
                return false;
        }
        return middle_found;
    }

    int raw_read(size_t sensor_index = 0)
    {
        return analogRead(sensor_pins[sensor_index]);
    }

    bool read_line_error(int &error)
    {
        long weighted_position = 0;
        int detected_sensors = 0;

        for (size_t index = 0; index < N; ++index)
        {
            if (!detects_line(sensor_pins[index]))
                continue;

            const long position = N == 1
                                      ? 0
                                      : -1000L + (2000L * index) / (N - 1);
            weighted_position += position;
            ++detected_sensors;
        }

        if (detected_sensors == 0)
            return false;

        error = weighted_position / detected_sensors;
        return true;
    }

    bool intersection_detected()
    {
        return left_detected() && right_detected();
    }

    State read()
    {
        const bool left = left_detected();
        const bool middle = middle_detected();
        const bool right = right_detected();
        const bool intersection = intersection_detected();

        if (intersection)
            return INTERSECTION;
        if (middle)
            return MIDDLE;
        if (left)
            return LEFT;
        if (right)
            return RIGHT;
        return NONE;
    }
};

#endif
