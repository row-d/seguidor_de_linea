#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include "component.h"

// Registro por un puerto serie de la Mega: Serial (USB) o Serial1 (Bluetooth).
// Eventos y telemetria solo se envian mientras hay un registro activo (set_recording).
// event() se puede llamar desde cualquier tarea: deja el evento en una cola sin tocar
// el puerto, y flush_events() lo escribe.
class Logger : public Component
{
private:
    struct LogEvent
    {
        unsigned long ms;
        const char *name;
    };

    // Eventos que caben en espera entre dos llamadas a flush_events().
    static const uint8_t EVENT_QUEUE_LENGTH = 8;

    HardwareSerial &port;
    unsigned long baud_rate;
    QueueHandle_t events = nullptr;
    volatile bool recording = false;

public:
    Logger(HardwareSerial &serial_port, unsigned long baud = 9600)
        : port(serial_port), baud_rate(baud) {}

    void setup() override
    {
        port.begin(baud_rate);
        events = xQueueCreate(EVENT_QUEUE_LENGTH, sizeof(LogEvent));
    }

    template <typename T>
    void print(T value)
    {
        port.print(value);
    }

    template <typename T>
    void println(T value)
    {
        port.println(value);
    }

    // Numero con 4 decimales: print() solo muestra 2, y Kd suele ser menor que 0.01.
    void print_decimal(float value)
    {
        port.print(value, 4);
    }

    // Siguiente caracter recibido, o -1 si no hay ninguno.
    int read()
    {
        return port.read();
    }

    bool is_recording() const
    {
        return recording;
    }

    // Formato CSV: recording,<ms>,start o recording,<ms>,stop. Detener sin registro
    // activo no hace nada.
    void set_recording(bool on)
    {
        if (!on && !recording)
            return;
        recording = on;
        port.print("recording,");
        port.print(millis());
        port.println(on ? ",start" : ",stop");
    }

    // Encola un evento con la hora actual. Si la cola esta llena, el evento se pierde.
    // name debe ser un literal: solo se guarda el puntero.
    void event(const char *name)
    {
        if (!recording || events == nullptr)
            return;

        const LogEvent entry = {millis(), name};
        xQueueSend(events, &entry, 0);
    }

    // Escribe los eventos en espera. Formato CSV: event,<ms>,<nombre>
    void flush_events()
    {
        LogEvent entry;
        while (events != nullptr && xQueueReceive(events, &entry, 0) == pdTRUE)
        {
            port.print("event,");
            port.print(entry.ms);
            port.print(',');
            port.println(entry.name);
        }
    }

    // Formato CSV: t,<ms>,<valores>. ms es la hora en que se tomo la muestra.
    void telemetry(unsigned long ms, const int *values, size_t count)
    {
        if (!recording)
            return;

        port.print("t,");
        port.print(ms);
        for (size_t index = 0; index < count; ++index)
        {
            port.print(',');
            port.print(values[index]);
        }
        port.println();
    }
};

#endif // LOGGER_H
