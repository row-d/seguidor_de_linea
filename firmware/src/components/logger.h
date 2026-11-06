#ifndef LOGGER_H
#define LOGGER_H

#include "component.h"

// Registro por un puerto serie de la Mega: Serial (USB) o Serial1 (Bluetooth).
// Eventos y telemetria solo se envian mientras hay un registro activo, que se
// controla desde el PC con los comandos START_COMMAND y STOP_COMMAND.
class Logger : public Component
{
public:
    static const char START_COMMAND = 'r';
    static const char STOP_COMMAND = 's';

private:
    HardwareSerial &port;
    unsigned long baud_rate;
    bool recording = false;

public:
    Logger(HardwareSerial &serial_port, unsigned long baud = 9600)
        : port(serial_port), baud_rate(baud) {}

    void setup() override
    {
        port.begin(baud_rate);
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

    bool is_recording() const
    {
        return recording;
    }

    // Lee los comandos pendientes del PC. Devuelve true si acaba de empezar un
    // registro, para que quien llama envie la cabecera.
    bool poll_commands()
    {
        bool started = false;
        while (port.available() > 0)
        {
            const char command = port.read();
            if (command == START_COMMAND)
            {
                recording = true;
                started = true;
                port.print("recording,");
                port.print(millis());
                port.println(",start");
            }
            else if (command == STOP_COMMAND && recording)
            {
                recording = false;
                port.print("recording,");
                port.print(millis());
                port.println(",stop");
            }
        }
        return started;
    }

    // Formato CSV: event,<ms>,<nombre>
    void event(const char *name)
    {
        if (!recording)
            return;

        port.print("event,");
        port.print(millis());
        port.print(',');
        port.println(name);
    }

    // Formato CSV: t,<ms>,<valores>. Descarta la fila si no cabe en el buffer de
    // salida, para no frenar el control esperando al puerto serie.
    void telemetry(const int *values, size_t count)
    {
        const size_t max_length = 15 + 6 * count;
        if (!recording || static_cast<size_t>(port.availableForWrite()) < max_length)
            return;

        port.print("t,");
        port.print(millis());
        for (size_t index = 0; index < count; ++index)
        {
            port.print(',');
            port.print(values[index]);
        }
        port.println();
    }
};

#endif // LOGGER_H
