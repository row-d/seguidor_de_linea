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
    int line_thresholds[N];
    const int (&sensor_pins)[N];

    bool detects_line(size_t sensor_index)
    {
        return analogRead(sensor_pins[sensor_index]) >= line_thresholds[sensor_index];
    }

    int line_strength(size_t sensor_index)
    {
        const int strength = analogRead(sensor_pins[sensor_index]) - line_thresholds[sensor_index];
        return strength > 0 ? strength : 0;
    }

    bool only_sensor(size_t sensor_index)
    {
        for (size_t index = 0; index < N; ++index)
        {
            if (index == sensor_index)
                continue;
            if (detects_line(index))
                return false;
        }
        return detects_line(sensor_index);
    }

public:
    LineSensors(const int (&pins)[N], int threshold = 130) : sensor_pins(pins)
    {
        for (size_t index = 0; index < N; ++index)
            line_thresholds[index] = threshold;
    }

    LineSensors(const int (&pins)[N], const int (&thresholds)[N]) : sensor_pins(pins)
    {
        for (size_t index = 0; index < N; ++index)
            line_thresholds[index] = thresholds[index];
    }

    void setup()
    {
        for (size_t i = 0; i < N; ++i)
        {
            pinMode(sensor_pins[i], INPUT_PULLUP);
        }
    }

    bool left_detected()
    {
        return only_sensor(0);
    }

    bool right_detected()
    {
        return only_sensor(N - 1);
    }

    bool middle_detected()
    {
        const size_t middle_index = N / 2;
        if (N % 2 == 1)
        {
            return only_sensor(middle_index);
        }
        else
        {
            for (size_t index = 0; index < N; ++index)
            {
                if (index == middle_index || index == middle_index - 1)
                    continue;
                if (detects_line(index))
                    return false;
            }

            return detects_line(middle_index) || detects_line(middle_index - 1);
        }
    }

    int raw_read(size_t sensor_index = 0)
    {
        return analogRead(sensor_pins[sensor_index]);
    }

    bool sensor_detected(size_t sensor_index)
    {
        return detects_line(sensor_index);
    }

    int central_raw()
    {
        return raw_read(N / 2);
    }

    int central_threshold()
    {
        return line_thresholds[N / 2];
    }

    bool read_line_error(int &error)
    {
        long weighted_position = 0;
        long total_strength = 0;

        for (size_t index = 0; index < N; ++index)
        {
            const int strength = line_strength(index);
            if (strength == 0)
                continue;

            const long position = N == 1
                                      ? 0
                                      : -1000L + (2000L * index) / (N - 1);
            weighted_position += position * strength;
            total_strength += strength;
        }

        if (total_strength == 0)
            return false;

        error = weighted_position / total_strength;
        return true;
    }

    bool intersection_detected()
    {
        return detects_line(0) && detects_line(N - 1) && !middle_detected();
    }

    bool all_detected()
    {
        for (size_t index = 0; index < N; ++index)
        {
            if (!detects_line(index))
                return false;
        }
        return true;
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
