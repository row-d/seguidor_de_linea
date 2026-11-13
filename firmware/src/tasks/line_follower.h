#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include <PID_v1.h>
#include "robot.h"

// Velocidad con signo de cada lado, lista para Movement::drive().
struct WheelSpeeds
{
    int left;
    int right;
    // Salida del PID, para la telemetria. Positiva gira a la derecha.
    int correction;
};

// Direccion del seguimiento de linea: convierte la posicion de la linea en la velocidad
// de cada rueda. La tarea de control (start.h) decide que hacer en bifurcaciones, metas
// y callejones; esta clase solo sabe mantenerse sobre la linea y buscarla si se pierde.
//
// En cada ciclo, get_speeds() hace tres cosas:
//   1. PID: corrige segun el error (linea a la izquierda o a la derecha).
//   2. Serpenteo: con la linea solo bajo el sensor central, oscila para no perderla.
//   3. Busqueda: sin linea, gira sobre su eje hacia el lado donde la vio por ultima vez.
class LineFollower
{
private:
    // Menor que el periodo real del control (~32 ms) para que Compute() actue en cada ciclo.
    static const int PID_SAMPLE_TIME_MS = 25;
    // Correccion maxima del PID: con una velocidad base de 130 deja una rueda en -70 y la otra al tope.
    static const int MAX_CORRECTION = 200;
    // Centrado: con |error| >= EDGE_ERROR la linea esta solo bajo un sensor lateral. El
    // robot deja de avanzar y gira hacia ese lado hasta que la linea llega al sensor
    // central. Si avanzara mientras corrige, en una curva seguiria la linea con el
    // sensor lateral y nunca la traeria al centro.
    static const int EDGE_ERROR = 800;

    // Serpenteo: los sensores estan mas separados que el ancho de la linea, asi que con
    // la linea solo bajo el central el error es 0 y el robot no sabe hacia donde se le
    // escapa. Mientras |error| < ZIGZAG_MAX_ERROR, alterna un giro suave a cada lado
    // para que un sensor lateral la encuentre antes de perderla.
    static const int ZIGZAG_MAX_ERROR = 100;
    // Diferencia de velocidad que se suma a una rueda y se resta a la otra.
    static const int ZIGZAG_SPEED = 50;
    // Tiempo girando hacia cada lado.
    static const unsigned long ZIGZAG_TIME_MS = 250;

    // Variables que PID_v1 lee y escribe a traves de punteros.
    double line_error = 0;
    double pid_output = 0;
    double target = 0;
    PID line_pid;

    // Lado hacia el que girar si la linea se pierde: -1 izquierda, 1 derecha.
    int search_side = -1;
    bool zigzag_on = false;
    unsigned long zigzag_start_time = 0;

    // Giro extra del serpenteo para este ciclo. Positivo gira a la derecha, igual que
    // la correccion del PID.
    int zigzag_turn(bool active)
    {
        if (!active)
        {
            zigzag_on = false;
            return 0;
        }

        if (!zigzag_on)
        {
            zigzag_on = true;
            // Empieza a mitad de un tramo para oscilar centrado sobre la linea.
            zigzag_start_time = millis() - ZIGZAG_TIME_MS / 2;
        }
        const bool first_side = ((millis() - zigzag_start_time) / ZIGZAG_TIME_MS) % 2 == 0;
        const int offset = first_side ? ZIGZAG_SPEED : -ZIGZAG_SPEED;
        // Si la linea se pierde girando hacia un lado, quedo del otro.
        search_side = offset > 0 ? -1 : 1;
        return offset;
    }

public:
    // PID_v1 divide Kd entre el periodo en segundos, por eso Kd es mucho menor que Kp.
    LineFollower(float kp, float ki, float kd)
        : line_pid(&line_error, &pid_output, &target, kp, ki, kd, REVERSE)
    {
        line_pid.SetOutputLimits(-MAX_CORRECTION, MAX_CORRECTION);
        line_pid.SetSampleTime(PID_SAMPLE_TIME_MS);
        line_pid.SetMode(AUTOMATIC);
    }

    // El PID guarda punteros a las variables de este objeto: no se puede copiar.
    LineFollower(const LineFollower &) = delete;
    LineFollower &operator=(const LineFollower &) = delete;

    // Cambia las ganancias sin reiniciar el PID: valen desde el siguiente get_speeds().
    void set_gains(float kp, float ki, float kd)
    {
        line_pid.SetTunings(kp, ki, kd);
    }

    // Fija hacia donde buscar la linea si se pierde: -1 izquierda, 1 derecha. La tarea
    // de control lo usa despues de girar en una bifurcacion.
    void search_to(int direction)
    {
        search_side = direction;
    }

    // Calcula la velocidad de cada rueda para este ciclo.
    //   line_detected, error: resultado de LineSensors::read_line_error().
    //   in_start_zone: en la zona de inicio todavia no hay linea que buscar; sin linea
    //   el robot avanza recto en vez de girar.
    WheelSpeeds get_speeds(bool line_detected, int error, bool in_start_zone)
    {
        if (line_detected)
        {
            line_error = error;

            if (error < 0)
                search_side = -1;
            else if (error > 0)
                search_side = 1;

            // Al volver a AUTOMATIC, PID_v1 arranca desde pid_output (0): no arrastra
            // lo acumulado antes de perder la linea.
            line_pid.SetMode(AUTOMATIC);
            line_pid.Compute();
        }
        else
        {
            // Sin linea el PID se apaga y su salida vuelve a 0. Si no, al encontrarla
            // seguia corrigiendo un rato hacia el lado viejo, alejandose de ella.
            line_pid.SetMode(MANUAL);
            pid_output = 0;
        }
        const int correction = static_cast<int>(pid_output);
        const int zigzag = zigzag_turn(line_detected && !in_start_zone && abs(error) < ZIGZAG_MAX_ERROR);

        WheelSpeeds speeds;
        speeds.correction = correction;
        if (line_detected && abs(error) >= EDGE_ERROR)
        {
            // Mismo giro que la busqueda: si la linea se pierde en el hueco entre el
            // sensor lateral y el central, el robot sigue girando hasta el central.
            turn_speeds(error < 0 ? -1 : 1, speeds.left, speeds.right);
        }
        else if (line_detected)
        {
            speeds.left = speed_settings.base + correction + zigzag;
            speeds.right = speed_settings.base - correction - zigzag;
            if (!tank_turn)
            {
                // Sin fuerza para girar sobre su eje: la rueda interior se detiene
                // en vez de ir hacia atras.
                speeds.left = max(speeds.left, 0);
                speeds.right = max(speeds.right, 0);
            }
        }
        else if (in_start_zone)
        {
            speeds.left = speed_settings.base;
            speeds.right = speed_settings.base;
        }
        else
        {
            turn_speeds(search_side, speeds.left, speeds.right);
        }
        return speeds;
    }
};

#endif
