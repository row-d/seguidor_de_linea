#ifndef LINE_SENSORS_H
#define LINE_SENSORS_H
#include "component.h"

template <size_t N>
class LineSensors : public Component
{
    static_assert(N > 0, "At least one line sensor is required");
public:
    enum State { NONE, LEFT, RIGHT, MIDDLE, INTERSECTION };
    struct Sample
    {
        int raw[N];
        int strength[N]; // 0..1000; comparable after calibration.
        bool detected[N];
        int error;
        bool found;
        bool ambiguous;
        bool valid;
        unsigned long captured_ms;
    };

private:
    const int (&sensor_pins)[N];
    int line_thresholds[N];
    int background[N] = {};
    int black[N] = {};
    bool calibrated = false;
    bool black_is_high;

public:
    LineSensors(const int (&pins)[N], int threshold = 130, bool high_on_black = true)
        : sensor_pins(pins), black_is_high(high_on_black)
    {
        for (size_t i = 0; i < N; ++i)
            line_thresholds[i] = threshold;
    }

    LineSensors(const int (&pins)[N], const int (&thresholds)[N], bool high_on_black = true)
        : sensor_pins(pins), black_is_high(high_on_black)
    {
        for (size_t i = 0; i < N; ++i)
            line_thresholds[i] = thresholds[i];
    }

    // Reject weak contrast without replacing the previous calibration.
    bool set_calibration(const int (&background_values)[N], const int (&black_values)[N])
    {
        for (size_t i = 0; i < N; ++i)
        {
            if (background_values[i] < 0 || background_values[i] > 1023 ||
                black_values[i] < 0 || black_values[i] > 1023 ||
                abs(black_values[i] - background_values[i]) < 50)
                return false;
        }
        for (size_t i = 0; i < N; ++i)
        {
            background[i] = background_values[i];
            black[i] = black_values[i];
            line_thresholds[i] = (background[i] + black[i]) / 2;
        }
        calibrated = true;
        return true;
    }

    void setup() override
    {
        for (size_t i = 0; i < N; ++i)
            pinMode(sensor_pins[i], INPUT);
    }

    // Only the acquisition task calls this during normal operation.
    Sample sample()
    {
        Sample result = {};
        result.valid = true;
        long weighted = 0;
        long total = 0;
        for (size_t i = 0; i < N; ++i)
        {
            const int raw = analogRead(sensor_pins[i]);
            result.raw[i] = raw;
            if (raw < 0 || raw > 1023)
                result.valid = false;
            int strength;
            if (calibrated)
            {
                strength = constrain((raw - background[i]) * 1000L /
                                         (black[i] - background[i]), 0L, 1000L);
                result.detected[i] = strength >= 500;
            }
            else
            {
                const int delta = black_is_high ? raw - line_thresholds[i]
                                                : line_thresholds[i] - raw;
                const int range = black_is_high ? 1023 - line_thresholds[i]
                                                : line_thresholds[i];
                strength = range > 0 ? constrain(delta * 1000L / range, 0L, 1000L) : 0;
                result.detected[i] = delta > 0;
            }
            result.strength[i] = result.detected[i] ? strength : 0;
            const long position = N == 1 ? 0 : -1000L + (2000L * i) / (N - 1);
            weighted += position * result.strength[i];
            total += result.strength[i];
        }
        result.found = result.valid && total > 0;
        result.error = total > 0 ? weighted / total : 0;
        result.ambiguous = N > 1 && result.detected[0] && result.detected[N - 1];
        result.captured_ms = millis();
        return result;
    }

    // Compatibility helpers. Tasks consume Sample instead of rereading the ADC.
    int raw_read(size_t i = 0) { return i < N ? analogRead(sensor_pins[i]) : -1; }
    int central_raw() { return raw_read(N / 2); }
    int central_threshold() { return line_thresholds[N / 2]; }
    bool sensor_detected(size_t i) { return i < N && sample().detected[i]; }
    bool read_line_error(int &error)
    {
        const Sample value = sample();
        if (value.found) error = value.error;
        return value.found;
    }
    bool intersection_detected() { return sample().ambiguous; }
    bool all_detected()
    {
        const Sample value = sample();
        for (size_t i = 0; i < N; ++i)
            if (!value.detected[i]) return false;
        return true;
    }
    State read()
    {
        const Sample value = sample();
        if (!value.valid || !value.found) return NONE;
        if (value.ambiguous) return INTERSECTION;
        if (value.error < -250) return LEFT;
        if (value.error > 250) return RIGHT;
        return MIDDLE;
    }
    bool left_detected() { return read() == LEFT; }
    bool right_detected() { return read() == RIGHT; }
    bool middle_detected() { return read() == MIDDLE; }
};

#endif
