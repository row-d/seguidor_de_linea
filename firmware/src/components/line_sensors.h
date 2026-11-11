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

    // Niveles en milesimas del rango fondo-linea de cada sensor (ver line_level).
    // Por debajo de NOISE_LEVEL una lectura es ruido del fondo y no cuenta.
    static const int NOISE_LEVEL = 100;
    // Si ningun sensor llega a este nivel, la linea esta perdida.
    static const int LINE_MIN_LEVEL = 180;

private:
    // Lectura a partir de la cual un sensor esta claramente sobre la linea: se usa
    // para reconocer bifurcaciones y lineas horizontales, no para seguir la linea.
    int line_thresholds[N];
    // Lectura de cada sensor sobre el fondo y sobre el centro de la linea.
    int white_levels[N];
    int black_levels[N];
    const int (&sensor_pins)[N];

    bool detects_line(size_t sensor_index)
    {
        return analogRead(sensor_pins[sensor_index]) >= line_thresholds[sensor_index];
    }

    // Cuanto ve la linea un sensor: 0 sobre el fondo, 1000 con la linea centrada.
    // A diferencia del umbral, tambien mide una linea que solo roza el sensor.
    int line_level(size_t sensor_index)
    {
        const long range = black_levels[sensor_index] - white_levels[sensor_index];
        if (range <= 0)
            return 0;

        const long level = (analogRead(sensor_pins[sensor_index]) - white_levels[sensor_index]) * 1000L / range;
        return constrain(level, 0, 1000);
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
            set_threshold(index, threshold);
    }

    LineSensors(const int (&pins)[N], const int (&thresholds)[N]) : sensor_pins(pins)
    {
        for (size_t index = 0; index < N; ++index)
            set_threshold(index, thresholds[index]);
    }

    void setup()
    {
        for (size_t i = 0; i < N; ++i)
        {
            // Sin pull-up: con un cable flojo la lectura subia sola y parecia linea.
            pinMode(sensor_pins[i], INPUT);
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

    // Fija el umbral sin conocer los niveles: los estima suponiendo un fondo en 0.
    void set_threshold(size_t sensor_index, int threshold)
    {
        line_thresholds[sensor_index] = threshold;
        white_levels[sensor_index] = 0;
        black_levels[sensor_index] = threshold * 3;
    }

    // Fija los niveles medidos en la calibracion. El umbral queda a un tercio del
    // rango y no a la mitad: black se mide con el sensor justo encima de la linea,
    // pero al seguirla la lectura baja bastante.
    void set_levels(size_t sensor_index, int white, int black)
    {
        white_levels[sensor_index] = white;
        black_levels[sensor_index] = black;
        line_thresholds[sensor_index] = white + (black - white) / 3;
    }

    int white_level(size_t sensor_index)
    {
        return white_levels[sensor_index];
    }

    int black_level(size_t sensor_index)
    {
        return black_levels[sensor_index];
    }

    int threshold(size_t sensor_index)
    {
        return line_thresholds[sensor_index];
    }

    // Posicion de la linea: -1000 bajo el sensor izquierdo, 0 en el centro y 1000 bajo
    // el derecho. Pondera cada sensor por su nivel, asi una linea que queda entre dos
    // sensores da un error intermedio en vez de perderse. Devuelve false sin linea.
    bool read_line_error(int &error)
    {
        long weighted_position = 0;
        long total_strength = 0;
        int strongest = 0;

        for (size_t index = 0; index < N; ++index)
        {
            const int strength = line_level(index);
            if (strength > strongest)
                strongest = strength;
            if (strength < NOISE_LEVEL)
                continue;

            const long position = N == 1
                                      ? 0
                                      : -1000L + (2000L * index) / (N - 1);
            weighted_position += position * strength;
            total_strength += strength;
        }

        if (strongest < LINE_MIN_LEVEL)
            return false;

        error = weighted_position / total_strength;
        return true;
    }

    bool intersection_detected()
    {
        const bool center = N % 2 == 1
                                ? detects_line(N / 2)
                                : detects_line(N / 2) || detects_line(N / 2 - 1);
        return detects_line(0) && detects_line(N - 1) && !center;
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
