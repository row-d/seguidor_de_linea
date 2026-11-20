#ifndef HC05_H
#define HC05_H

#include "component.h"

// Configura un HC-05 con comandos AT. Las respuestas se muestran por output.
class Hc05 : public Component
{
private:
    // Velocidad fija del modo AT cuando el modulo arranca con el boton pulsado.
    static const unsigned long AT_BAUD = 38400;
    static const unsigned long REPLY_TIMEOUT_MS = 1000;

    HardwareSerial &port;
    Print &output;

    // Envia "<prefix><value><suffix>" al modulo, lo muestra por output y espera la respuesta.
    template <typename T>
    bool send(const char *prefix, T value, const char *suffix)
    {
        Print *targets[] = {&output, &port};
        for (Print *target : targets)
        {
            target->print(prefix);
            target->print(value);
            target->println(suffix);
        }
        return read_reply();
    }

    bool read_reply()
    {
        char line[32];
        size_t length = 0;
        const unsigned long start = millis();

        while (millis() - start < REPLY_TIMEOUT_MS)
        {
            const int received = port.read();
            if (received == '\n')
            {
                line[length] = '\0';
                output.println(line);
                if (strcmp(line, "OK") == 0)
                    return true;
                if (strncmp(line, "ERROR", 5) == 0)
                    return false;
                length = 0;
            }
            else if (received >= 0 && received != '\r' && length < sizeof(line) - 1)
            {
                line[length++] = received;
            }
        }

        output.println("sin respuesta");
        return false;
    }

public:
    Hc05(HardwareSerial &serial_port, Print &output_port)
        : port(serial_port), output(output_port) {}

    void setup() override
    {
        port.begin(AT_BAUD);
    }

    bool configure(const char *name, unsigned long baud)
    {
        return send("AT", "", "") &&
               send("AT+NAME=", name, "") &&
               send("AT+UART=", baud, ",0,0");
    }
};

#endif // HC05_H
