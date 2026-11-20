#ifndef COMMANDS_H
#define COMMANDS_H

#include "../components/logger.h"

// Comandos de una letra que el PC envia por el puerto del Logger (scripts/console.ps1).
// GAINS y SPEEDS llevan una linea de argumentos: "p0.25,0,0.005\n" (comas o espacios).
namespace commands
{
    // Deben coincidir con las letras de scripts/console.ps1.
    const char RECORD = 'r';
    const char STOP_RECORD = 's';
    const char CALIBRATE = 'c';
    const char RUN = 'g';
    const char HALT = 'x';
    const char GAINS = 'p';
    const char SPEEDS = 'b';

    // Lo que debe hacer quien llama a poll(). Se pueden combinar.
    const uint8_t REQUEST_HEADER = 1; // empezo un registro: enviar la cabecera
    const uint8_t REQUEST_CALIBRATION = 2;
    const uint8_t REQUEST_RUN = 4;
    const uint8_t REQUEST_HALT = 8;
    // Comando con argumentos (values_command dice cual). Con REQUEST_VALUES_SET, los
    // numeros recibidos estan en values.
    const uint8_t REQUEST_VALUES_SET = 16;
    const uint8_t REQUEST_VALUES_QUERY = 32; // linea vacia: consultar los valores en uso
    const uint8_t REQUEST_VALUES_INVALID = 64;

    // Si la linea de argumentos no termina en este tiempo, se descarta: asi una linea
    // cortada no se traga los comandos siguientes (p. ej. detener).
    const unsigned long ARGUMENT_TIMEOUT_MS = 2000;

    char values_command = GAINS; // comando con argumentos en curso o el ultimo recibido
    float values[3] = {0, 0, 0};
    char arguments[32];
    uint8_t argument_length = 0;
    bool reading_arguments = false;
    unsigned long arguments_started_at = 0;

    uint8_t parse_values()
    {
        arguments[argument_length] = '\0';
        if (argument_length == 0)
            return REQUEST_VALUES_QUERY;

        char *cursor = arguments;
        for (float &value : values)
        {
            char *end = cursor;
            value = strtod(cursor, &end);
            if (end == cursor)
                return REQUEST_VALUES_INVALID;
            cursor = end;
            while (*cursor == ',' || *cursor == ' ')
                ++cursor;
        }
        return *cursor == '\0' ? REQUEST_VALUES_SET : REQUEST_VALUES_INVALID;
    }

    uint8_t read_argument(char received)
    {
        if (received == '\n' || received == '\r')
        {
            reading_arguments = false;
            return parse_values();
        }
        if (argument_length < sizeof(arguments) - 1)
        {
            arguments[argument_length++] = received;
            return 0;
        }
        reading_arguments = false; // linea demasiado larga
        return REQUEST_VALUES_INVALID;
    }

    uint8_t read_command(Logger &logger, char received)
    {
        switch (received)
        {
        case GAINS:
        case SPEEDS:
            values_command = received;
            reading_arguments = true;
            argument_length = 0;
            arguments_started_at = millis();
            return 0;
        case CALIBRATE:
            return REQUEST_CALIBRATION;
        case RUN:
            return REQUEST_RUN;
        case HALT:
            return REQUEST_HALT;
        case RECORD:
            logger.set_recording(true);
            return REQUEST_HEADER;
        case STOP_RECORD:
            logger.set_recording(false);
            return 0;
        default:
            return 0;
        }
    }

    // Lee los comandos pendientes del PC y devuelve los REQUEST_* que piden.
    uint8_t poll(Logger &logger)
    {
        uint8_t requests = 0;
        if (reading_arguments && millis() - arguments_started_at > ARGUMENT_TIMEOUT_MS)
        {
            reading_arguments = false;
            requests |= REQUEST_VALUES_INVALID;
        }

        int received;
        while ((received = logger.read()) >= 0)
            requests |= reading_arguments ? read_argument(received) : read_command(logger, received);
        return requests;
    }
}

#endif // COMMANDS_H
