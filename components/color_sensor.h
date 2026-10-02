#ifndef COLOR_SENSOR_H
#define COLOR_SENSOR_H

#include "component.h"

struct RgbColor
{
    uint16_t red;
    uint16_t green;
    uint16_t blue;
};

class ColorSensor : public Component
{
public:
    typedef void (*Reader)(RgbColor &color);

private:
    Reader reader;
    RgbColor target;
    uint16_t tolerance;

public:
    ColorSensor(Reader color_reader,
                RgbColor target_color,
                uint16_t color_tolerance)
        : reader(color_reader), target(target_color), tolerance(color_tolerance) {}

    void setup() {}

    RgbColor read()
    {
        RgbColor color{0, 0, 0};
        if (reader != nullptr)
        {
            reader(color);
        }
        return color;
    }

    bool object_detected()
    {
        const RgbColor color = read();
        return within_tolerance(color.red, target.red) &&
               within_tolerance(color.green, target.green) &&
               within_tolerance(color.blue, target.blue);
    }

private:
    bool within_tolerance(uint16_t value, uint16_t expected) const
    {
        const uint16_t difference = value > expected ? value - expected : expected - value;
        return difference <= tolerance;
    }
};

#endif
