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

    bool read_reply()
    {
        char line[32];
        size_t length = 0;
        const unsigned long start = millis();

        while (millis() - start < REPLY_TIMEOUT_MS)
        {
            if (!port.available())
                continue;

            const char received = port.read();
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
            else if (received != '\r' && length < sizeof(line) - 1)
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
        output.println("AT");
        port.println("AT");
        if (!read_reply())
            return false;

        output.print("AT+NAME=");
        output.println(name);
        port.print("AT+NAME=");
        port.println(name);
        if (!read_reply())
            return false;

        output.print("AT+UART=");
        output.print(baud);
        output.println(",0,0");
        port.print("AT+UART=");
        port.print(baud);
        port.println(",0,0");
        return read_reply();
    }
};

#endif // HC05_H
