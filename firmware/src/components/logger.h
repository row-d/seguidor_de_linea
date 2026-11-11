#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include "component.h"

// Registro por un puerto serie de la Mega: Serial (USB) o Serial1 (Bluetooth).
// Eventos y telemetria solo se envian mientras hay un registro activo, que se
// controla desde el PC con los comandos START_COMMAND y STOP_COMMAND.
//
// Solo la tarea de telemetria lee y escribe el puerto. Las demas tareas llaman a
// event(), que deja el evento en una cola sin tocar el puerto.
class Logger : public Component
{
public:
    static const char START_COMMAND = 'r';
    static const char STOP_COMMAND = 's';
    static const char CALIBRATE_COMMAND = 'c';
    static const char RUN_COMMAND = 'g';
    static const char HALT_COMMAND = 'x';
    // Unico comando con argumentos: "p<kp>,<ki>,<kd>" y fin de linea cambia las
    // ganancias del PID; "p" y fin de linea las consulta.
    static const char GAINS_COMMAND = 'p';

    // Lo que debe hacer quien llama a poll_commands(). Se pueden combinar.
    static const uint8_t REQUEST_HEADER = 1;
    static const uint8_t REQUEST_CALIBRATION = 2;
    static const uint8_t REQUEST_RUN = 4;
    static const uint8_t REQUEST_HALT = 8;
    // Con REQUEST_GAINS_SET, los valores estan en gain_values().
    static const uint8_t REQUEST_GAINS_SET = 16;
    static const uint8_t REQUEST_GAINS_QUERY = 32;
    static const uint8_t REQUEST_GAINS_INVALID = 64;

private:
    struct LogEvent
    {
        unsigned long ms;
        const char *name;
    };

    // Eventos que caben en espera entre dos pasadas de la tarea de telemetria.
    static const uint8_t EVENT_QUEUE_LENGTH = 8;
    // Si la linea de un comando con argumentos no termina en este tiempo, se descarta:
    // asi una linea cortada no se traga los comandos siguientes (p. ej. detener).
    static const unsigned long ARGUMENT_TIMEOUT_MS = 2000;

    HardwareSerial &port;
    unsigned long baud_rate;
    QueueHandle_t events = nullptr;
    // La escribe la tarea de telemetria y la leen las demas.
    volatile bool recording = false;

    // Argumentos del comando GAINS_COMMAND que se esta recibiendo.
    char arguments[32];
    uint8_t argument_length = 0;
    bool reading_arguments = false;
    unsigned long arguments_started_at = 0;
    float gains[3] = {0, 0, 0};

    // Convierte los argumentos recibidos ("0.25,0,0.005", con comas o espacios).
    uint8_t parse_gains()
    {
        arguments[argument_length] = '\0';
        if (argument_length == 0)
            return REQUEST_GAINS_QUERY;

        char *cursor = arguments;
        for (float &gain : gains)
        {
            char *end = cursor;
            gain = strtod(cursor, &end);
            if (end == cursor)
                return REQUEST_GAINS_INVALID;
            cursor = end;
            while (*cursor == ',' || *cursor == ' ')
                ++cursor;
        }
        return *cursor == '\0' ? REQUEST_GAINS_SET : REQUEST_GAINS_INVALID;
    }

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

    bool is_recording() const
    {
        return recording;
    }

    // Kp, Ki y Kd del ultimo REQUEST_GAINS_SET.
    const float *gain_values() const
    {
        return gains;
    }

    // Lee los comandos pendientes del PC. Devuelve REQUEST_HEADER si acaba de empezar
    // un registro, para que quien llama envie la cabecera, REQUEST_CALIBRATION si
    // el PC pidio calibrar los sensores, REQUEST_RUN si pidio empezar el recorrido y
    // REQUEST_HALT si pidio detenerlo. Para las ganancias del PID devuelve
    // REQUEST_GAINS_SET, REQUEST_GAINS_QUERY o REQUEST_GAINS_INVALID.
    uint8_t poll_commands()
    {
        uint8_t requests = 0;
        if (reading_arguments && millis() - arguments_started_at > ARGUMENT_TIMEOUT_MS)
        {
            reading_arguments = false;
            requests |= REQUEST_GAINS_INVALID;
        }

        while (port.available() > 0)
        {
            const char command = port.read();
            if (reading_arguments)
            {
                if (command == '\n' || command == '\r')
                {
                    reading_arguments = false;
                    requests |= parse_gains();
                }
                else if (argument_length < sizeof(arguments) - 1)
                {
                    arguments[argument_length++] = command;
                }
                else
                {
                    // Linea demasiado larga.
                    reading_arguments = false;
                    requests |= REQUEST_GAINS_INVALID;
                }
                continue;
            }

            if (command == GAINS_COMMAND)
            {
                reading_arguments = true;
                argument_length = 0;
                arguments_started_at = millis();
            }
            else if (command == CALIBRATE_COMMAND)
            {
                requests |= REQUEST_CALIBRATION;
            }
            else if (command == RUN_COMMAND)
            {
                requests |= REQUEST_RUN;
            }
            else if (command == HALT_COMMAND)
            {
                requests |= REQUEST_HALT;
            }
            else if (command == START_COMMAND)
            {
                recording = true;
                requests |= REQUEST_HEADER;
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
        return requests;
    }

    // Encola un evento con la hora actual. Se puede llamar desde cualquier tarea:
    // no escribe en el puerto ni espera. Si la cola esta llena, el evento se pierde.
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
