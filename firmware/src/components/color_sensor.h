#ifndef COLOR_SENSOR_H
#define COLOR_SENSOR_H

#include "component.h"

struct RgbColor
{
    uint16_t red;
    uint16_t green;
    uint16_t blue;

    bool close_to(const RgbColor &other, uint16_t tolerance) const
    {
        return near(red, other.red, tolerance) &&
               near(green, other.green, tolerance) &&
               near(blue, other.blue, tolerance);
    }

private:
    // Sin restar con signo: en AVR int es de 16 bits y no cabe un uint16_t.
    static bool near(uint16_t value, uint16_t expected, uint16_t tolerance)
    {
        return (value > expected ? value - expected : expected - value) <= tolerance;
    }
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
    ColorSensor(Reader color_reader, RgbColor target_color, uint16_t color_tolerance)
        : reader(color_reader), target(target_color), tolerance(color_tolerance) {}

    void setup() override {}

    RgbColor read()
    {
        RgbColor color{0, 0, 0};
        if (reader != nullptr)
            reader(color);
        return color;
    }

    bool object_detected()
    {
        return read().close_to(target, tolerance);
    }
};

#endif
