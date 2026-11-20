#ifndef LINE_SENSORS_H
#define LINE_SENSORS_H
#include "component.h"

template <size_t N = 3>
class LineSensors : public Component
{
public:
    // line_error() da -MAX_ERROR bajo el primer sensor y MAX_ERROR bajo el ultimo.
    static const int MAX_ERROR = 2000;

    // Lecturas minima (fondo) y maxima (linea) de cada sensor durante una calibracion.
    struct Calibration
    {
        int min_read[N];
        int max_read[N];

        Calibration()
        {
            for (size_t i = 0; i < N; ++i)
            {
                min_read[i] = 1023;
                max_read[i] = 0;
            }
        }

        int threshold(size_t i) const
        {
            return min_read[i] + (max_read[i] - min_read[i]) / 3;
        }
    };

private:
    int line_thresholds[N];
    const int (&sensor_pins)[N];

public:
    LineSensors(const int (&pins)[N], const int (&thresholds)[N]) : sensor_pins(pins)
    {
        for (size_t i = 0; i < N; ++i)
            line_thresholds[i] = thresholds[i];
    }

    void setup()
    {
        for (size_t i = 0; i < N; ++i)
            pinMode(sensor_pins[i], INPUT);
    }

    int raw_read(size_t i)
    {
        return analogRead(sensor_pins[i]);
    }

    int threshold(size_t i) const
    {
        return line_thresholds[i];
    }

    bool detects_line(size_t i)
    {
        return raw_read(i) >= line_thresholds[i];
    }

    size_t count_on_line()
    {
        size_t count = 0;
        for (size_t i = 0; i < N; ++i)
            count += detects_line(i);
        return count;
    }

    // Lee todos los sensores: el valor de cada uno y si supera su umbral.
    void read(int (&values)[N], bool (&black)[N])
    {
        for (size_t i = 0; i < N; ++i)
        {
            values[i] = raw_read(i);
            black[i] = values[i] >= line_thresholds[i];
        }
    }

    // Posicion de la linea con los sensores marcados en black, ponderada por su lectura.
    // Una linea entre dos sensores da un error intermedio. Devuelve false sin linea.
    static bool line_error(const int (&values)[N], const bool (&black)[N], int &error)
    {
        // long: en AVR int es de 16 bits y value * 2000 desborda.
        long sum = 0;
        long total = 0;
        for (size_t i = 0; i < N; ++i)
        {
            const long weight = black[i] ? values[i] : 0;
            sum += (long)(2 * (int)i - (int)(N - 1)) * MAX_ERROR / (int)(N - 1) * weight;
            total += weight;
        }
        if (total == 0)
            return false;
        error = sum / total;
        return true;
    }

    void sample_calibration(Calibration &cal)
    {
        for (size_t i = 0; i < N; ++i)
        {
            const int value = raw_read(i);
            cal.min_read[i] = min(cal.min_read[i], value);
            cal.max_read[i] = max(cal.max_read[i], value);
        }
    }

    // Un sensor que casi no cambio no vio la linea: conserva su umbral.
    void apply_calibration(const Calibration &cal)
    {
        for (size_t i = 0; i < N; ++i)
        {
            if (cal.max_read[i] - cal.min_read[i] > 100)
                line_thresholds[i] = cal.threshold(i);
        }
    }
};

#endif
