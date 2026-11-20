#ifndef WHEEL_SENSOR_H
#define WHEEL_SENSOR_H

#include "component.h"

// Encoders DFRobot SEN0038 (20 PPR): https://wiki.dfrobot.com/sen0038/
// Cuenta los dos flancos con attachInterrupt(CHANGE), como el ejemplo de DFRobot,
// sin sentido de giro. Los pines deben admitir interrupcion
// (Mega: 2, 3, 18, 19, 20, 21). Solo puede haber un objeto WheelSensor.
template <size_t N>
class WheelSensor : public Component
{
public:
    WheelSensor(const int (&pins)[N], uint8_t pulses_per_rev = 20, uint16_t wheel_diameter_mm = 65)
        : pins(pins), pulses_per_rev(pulses_per_rev), wheel_diameter_mm(wheel_diameter_mm) {}

    void setup() override
    {
        instance = this;
        reset();
        attach_all(Index<0>());
    }

    void reset()
    {
        noInterrupts();
        for (size_t i = 0; i < N; ++i)
            edges[i] = period_us[i] = 0;
        interrupts();
    }

    // Flancos desde el ultimo reset(): 2 * pulses_per_rev por vuelta.
    unsigned long edge_count(size_t i) const
    {
        noInterrupts();
        const unsigned long count = edges[i];
        interrupts();
        return count;
    }

    // Segun el ultimo pulso completo; 0 si no hay flancos en 300 ms.
    float rpm(size_t i) const
    {
        noInterrupts();
        const unsigned long period = period_us[i];
        const unsigned long last = edge_us[i][(edges[i] + 1) & 1];
        interrupts();
        if (period == 0 || micros() - last > 300000UL)
            return 0;
        return 60000000.0f / ((float)period * pulses_per_rev);
    }

    float distance_mm(size_t i) const { return edge_count(i) * PI * wheel_diameter_mm / (2.0f * pulses_per_rev); }
    float speed_mm_s(size_t i) const { return rpm(i) / 60.0f * PI * wheel_diameter_mm; }

private:
    template <size_t I>
    struct Index {};

    static WheelSensor *instance;
    const int (&pins)[N];
    uint8_t pulses_per_rev;
    uint16_t wheel_diameter_mm;
    // Los flancos alternan subida/bajada: edge_us[i][edges & 1] guarda el del mismo tipo.
    volatile unsigned long edges[N], period_us[N], edge_us[N][2];

    void attach_all(Index<N>) {}

    template <size_t I>
    void attach_all(Index<I>)
    {
        pinMode(pins[I], INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(pins[I]), on_edge<I>, CHANGE);
        attach_all(Index<I + 1>());
    }

    template <size_t I>
    static void on_edge()
    {
        const unsigned long now = micros();
        WheelSensor &s = *instance;
        const uint8_t type = s.edges[I] & 1;
        if (s.edges[I] >= 2) // ya hay un flanco anterior del mismo tipo
            s.period_us[I] = now - s.edge_us[I][type];
        s.edge_us[I][type] = now;
        ++s.edges[I];
    }
};

template <size_t N>
WheelSensor<N> *WheelSensor<N>::instance = nullptr;

#endif // WHEEL_SENSOR_H
