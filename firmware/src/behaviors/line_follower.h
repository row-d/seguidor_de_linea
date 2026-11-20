#ifndef LINE_FOLLOWER_TASK_H
#define LINE_FOLLOWER_TASK_H

#include <Arduino_FreeRTOS.h>
#include <PID_v1.h>
#include "../components/line_sensors.h"
#include "../components/movement.h"
#include "../components/led_array.h"
#include "../components/buzzer.h"
#include "../components/logger.h"

// Sigue la linea con un PID sobre los 3 sensores: error 0 con la linea bajo el central;
// si un lateral la ve, el error crece hacia ese lado y el robot gira hacia el. Ademas
// resuelve la salida, los cruces, la llegada, las bifurcaciones y la linea perdida.
// Al llegar a la meta se para y toca la musica.
namespace line_follower
{
    const int PERIOD_MS = 15;
    // Mas que las demas tareas: stop() se llama desde una de menor prioridad.
    const UBaseType_t PRIORITY = 2;

    // Mas KP = zigzaguea en las rectas.
    const double KP = 0.152;
    const double KI = 0.00;
    const double KD = 0.001;
    // Hasta este error (linea entre el central y un lateral) va a la velocidad base; por
    // encima baja hasta la de curva. Asi las correcciones pequenas de las rectas no lo frenan.
    const int SLOWDOWN_FROM_ERROR = 1000;
    const int MAX_ERROR = LineSensors<3>::MAX_ERROR;
    // Velocidades del seguimiento con PID, -255..255. Se cambian con el comando b.
    struct Speeds
    {
        int base; // con la linea bajo el sensor central (error 0)
        // Base con la linea bajo un sensor lateral. Baja = en las curvas gira casi sobre su
        // eje en vez de avanzar y salirse (0 = gira sobre su eje).
        int curve;
        // Minima de la rueda interior al girar. 0 = se para y el robot gira sobre ella
        // empujando solo con la exterior: necesita menos fuerza que girar sobre su eje.
        // Negativo = va hacia atras (gira mas cerrado, pero si los motores no pueden
        // arrastrar las ruedas de lado el robot se queda parado).
        int inner_min;
    };
    Speeds speeds = {92, 0, 0};
    // Diferencia de velocidad entre ruedas por debajo de la cual los LEDs indican recto.
    const int STRAIGHT_DEADBAND = 30;

    const size_t SENSOR_LEFT = 0;
    const size_t SENSOR_CENTER = 1;
    const size_t SENSOR_RIGHT = 2;

    // Con los dos laterales en negro (salida, cruce o llegada) cruza recto a velocidad fija,
    // sin PID. Tiene que ser fija porque FINISH_CHECK_MS y FINISH_MS dependen de ella, y
    // baja para no pasarse mucho de la llegada mientras comprueba si la linea sigue.
    const int ALL_BLACK_SPEED = 80;
    // La linea horizontal sobre la que arranca es la salida: mientras siga encima no cuenta
    // como llegada, y la cruza a START_SPEED (mas que ALL_BLACK_SPEED para arrancar desde
    // parado).
    const int START_SPEED = 130;
    // Salida, cruces y llegada se ven igual (los 3 en negro un momento); cambia lo que hay
    // despues: tras la salida y los cruces la linea sigue, tras la llegada no. Si al salir
    // del negro no ve linea en FINISH_CHECK_MS, era la llegada: retrocede hasta quedar
    // encima y se para. Si da llegadas falsas tras un cruce, subirlo.
    const unsigned long FINISH_CHECK_MS = 100;
    // La linea perdida cuenta como "justo despues del negro" si empieza en este tiempo.
    const unsigned long AFTER_BLACK_MS = 100;
    const unsigned long BACK_TO_FINISH_TIMEOUT_MS = 1000;
    // Respaldo: los 3 en negro durante FINISH_MS seguidos (llegada con un bloque ancho).
    // Tiene que ser mas de lo que tarda en cruzar el cruce mas ancho a ALL_BLACK_SPEED.
    const unsigned long FINISH_MS = 600;
    // Tiempo que se perdona perder el todo-negro (ruido, uniones de cinta) sin reiniciar la cuenta.
    const unsigned long ALL_BLACK_TOLERANCE_MS = 40;

    // Linea perdida (los 3 en blanco): durante LOST_GRACE_MS sigue con las mismas
    // velocidades (en recta la linea suele haber quedado entre dos sensores y aparece
    // enseguida bajo un lateral; en curva sigue girando). Si no aparece, retrocede.
    const unsigned long LOST_GRACE_MS = 250;
    const int RECOVER_SPEED = 120;
    // Diferencia entre ruedas al retroceder, para que el morro gire hacia el lado donde
    // vio la linea por ultima vez. 0 = retrocede recto; igual a RECOVER_SPEED = la rueda
    // de fuera se para y el robot pivota sobre ella; mayor = gira casi sobre su eje.
    const int RECOVER_TURN = 120;

    // Bifurcaciones: el robot prefiere la rama de FORK_SIDE. Busca esa rama girando hacia
    // FORK_SIDE hasta FORK_FIRST_MS (como las ruedas van detras, los sensores barren hacia
    // ese lado) cuando:
    // - se pierde la linea viniendo centrado (el central cae en el hueco de una Y), o
    // - solo el lateral del otro lado ve la linea (con PRIORITIZE_FORK_SIDE): en una Y la
    //   rama de FORK_SIDE puede quedar tan lejos que solo se ve la otra.
    // Si la encuentra se queda con ella; si no, deshace el giro (el mismo movimiento marcha
    // atras) y sigue hacia el otro lado como en una curva normal.
    const int FORK_SIDE = 1; // 1 derecha, -1 izquierda
    // true: prioridad FORK_SIDE > recto > el otro lado en cualquier desvio. Cada curva hacia
    // el otro lado cuesta ~2 x FORK_FIRST_MS (busca, no encuentra y deshace).
    const bool PRIORITIZE_FORK_SIDE = true;
    const int FORK_SEARCH_SPEED = 220; // rueda exterior; la interior va a speeds.inner_min
    // Cuanto gira buscando la rama de FORK_SIDE: mas = la encuentra aunque este lejos.
    const unsigned long FORK_FIRST_MS = 350;
    // Perdida viniendo centrado, ya buscado hacia FORK_SIDE: gira hacia el otro lado como
    // mucho este tiempo; si tampoco aparece, retrocede.
    const unsigned long FORK_SECOND_MS = 500;
    // Tras encontrar la rama de FORK_SIDE, el lateral del otro lado no cuenta durante este
    // tiempo, para que la otra rama no lo desvie.
    const unsigned long FORK_COMMIT_MS = 400;
    const size_t SIDE_SENSOR = FORK_SIDE > 0 ? SENSOR_RIGHT : SENSOR_LEFT;
    const size_t OTHER_SENSOR = FORK_SIDE > 0 ? SENSOR_LEFT : SENSOR_RIGHT;

    const uint8_t SEARCH_NONE = 0;
    const uint8_t SEARCH_SIDE = 1; // girando hacia FORK_SIDE
    const uint8_t SEARCH_UNDO = 2; // deshaciendo ese giro (mismo movimiento marcha atras)

    struct Components
    {
        LineSensors<3> &sensors;
        Movement &wheels;
        LedArray<2> &turn_leds;
        Buzzer &buzzer;
        Logger &logger;
    };

    Components *hw = nullptr; // lo fija start()
    TaskHandle_t handle = nullptr; // nullptr si no hay un recorrido en marcha

    double pid_input = 0, pid_output = 0, pid_setpoint = 0;
    PID pid{&pid_input, &pid_output, &pid_setpoint, KP, KI, KD, REVERSE};

    // Estado de un recorrido: start() lo reinicia.
    struct State
    {
        bool at_start = true; // sigue sobre la linea de salida
        unsigned long start_black_last = 0;
        bool in_all_black = false;
        unsigned long all_black_start = 0;
        unsigned long all_black_last = 0;
        bool seen_all_black = false;
        bool lost = false;
        unsigned long lost_since = 0;
        bool centered = true;       // la ultima linea vista estaba bajo el central
        bool lost_centered = false; // se perdio viniendo centrado
        int last_side = -1;         // -1 izquierda, 1 derecha: donde vio la linea por ultima vez
        int last_speed_l = 0;       // ultimas velocidades con la linea a la vista
        int last_speed_r = 0;
        bool committed = false; // siguiendo la rama de FORK_SIDE
        unsigned long committed_since = 0;
        uint8_t search = SEARCH_NONE;
        unsigned long search_since = 0;
        unsigned long search_turned_ms = 0;
        // Tras buscar hacia FORK_SIDE sin encontrar nada, no vuelve a buscar hasta que el
        // central vea la linea (si no, en una curva hacia el otro lado buscaria sin parar).
        bool search_done = false;
    };
    State state;

    // Pasa a MANUAL: al volver a AUTOMATIC el PID se reinicia sin arrastrar la integral.
    void stop_pid()
    {
        pid.SetMode(MANUAL);
        pid_output = 0;
    }

    void drive(int speed_l, int speed_r)
    {
        hw->wheels.drive_compensated(speed_l, speed_r);
        hw->turn_leds.show_turn(speed_l, speed_r, STRAIGHT_DEADBAND);
    }

    // Avanza con la linea a la vista y guarda las velocidades para cuando se pierda.
    void drive_on_line(int speed_l, int speed_r)
    {
        state.last_speed_l = speed_l;
        state.last_speed_r = speed_r;
        drive(speed_l, speed_r);
    }

    // Gira hacia dir (1 derecha, -1 izquierda): rueda exterior a FORK_SEARCH_SPEED e
    // interior a speeds.inner_min. sense = -1 hace el mismo movimiento marcha atras, y en el
    // mismo tiempo deshace el giro.
    void search_pivot(int dir, int sense)
    {
        const int outer = sense * FORK_SEARCH_SPEED;
        const int inner = sense * max(-FORK_SEARCH_SPEED, speeds.inner_min);
        if (dir > 0)
            drive(outer, inner);
        else
            drive(inner, outer);
    }

    void start_search(unsigned long now)
    {
        state.search = SEARCH_SIDE;
        state.search_since = now;
        search_pivot(FORK_SIDE, 1);
    }

    // Retrocede recto hasta que al menos 2 sensores ven la linea de llegada (2 y no 3 por si
    // llega algo torcido) y se para encima.
    void back_to_finish()
    {
        drive(-ALL_BLACK_SPEED, -ALL_BLACK_SPEED);
        const unsigned long start = millis();
        while (millis() - start < BACK_TO_FINISH_TIMEOUT_MS && hw->sensors.count_on_line() < 2)
            vTaskDelay(1);
        hw->wheels.stop();
    }

    // Resultado de cada fase de step().
    enum Result : uint8_t
    {
        NEXT,     // no aplica: sigue la fase siguiente
        HANDLED,  // ya movio el robot en esta vuelta
        FINISHED, // llego a la meta
    };

    // Lectura de una vuelta. Las fases pueden borrar el negro de un lateral para ignorarlo.
    struct Reading
    {
        unsigned long now;
        int values[3];
        bool black[3];
    };

    bool both_sides_black(const Reading &r)
    {
        return r.black[SENSOR_LEFT] && r.black[SENSOR_RIGHT];
    }

    void mark_all_black(unsigned long now)
    {
        state.all_black_last = now;
        state.seen_all_black = true;
    }

    Result finish()
    {
        hw->logger.println("meta");
        return FINISHED;
    }

    // Recien tomada la rama de FORK_SIDE, el lateral del otro lado no cuenta.
    Result ignore_other_branch(Reading &r)
    {
        state.committed = state.committed && r.now - state.committed_since < FORK_COMMIT_MS;
        if (state.committed)
            r.black[OTHER_SENSOR] = false;
        return NEXT;
    }

    // Salida: hasta dejar la linea de salida (los dos laterales sin negro mas de
    // ALL_BLACK_TOLERANCE_MS, por si una lectura falla) avanza recto y no busca la meta.
    Result leave_start(Reading &r)
    {
        State &s = state;
        if (!s.at_start)
            return NEXT;
        if (both_sides_black(r))
            s.start_black_last = r.now;
        s.at_start = r.now - s.start_black_last <= ALL_BLACK_TOLERANCE_MS;
        if (!s.at_start)
            return NEXT;
        stop_pid();
        drive_on_line(START_SPEED, START_SPEED);
        return HANDLED;
    }

    // Busqueda en curso: gira hacia FORK_SIDE hasta FORK_FIRST_MS mirando solo el central
    // y el lateral de ese lado. Si la encuentra se queda con esa rama; si no, pasa a
    // deshacer el giro.
    Result search_side(Reading &r)
    {
        State &s = state;
        if (s.search != SEARCH_SIDE)
            return NEXT;
        stop_pid();
        if (r.black[SENSOR_CENTER] || r.black[SIDE_SENSOR])
        {
            s.search = SEARCH_NONE;
            s.committed = true;
            s.committed_since = r.now;
            r.black[OTHER_SENSOR] = false;
            s.lost = false;
            return NEXT;
        }
        if (r.now - s.search_since < FORK_FIRST_MS)
        {
            search_pivot(FORK_SIDE, 1);
            return HANDLED;
        }
        s.search = SEARCH_UNDO;
        s.search_turned_ms = r.now - s.search_since;
        s.search_since = r.now;
        return NEXT;
    }

    // Deshace el giro de la busqueda (el mismo movimiento marcha atras) y sigue con lo
    // que vea (normalmente, la linea del otro lado).
    Result undo_search(Reading &r)
    {
        State &s = state;
        if (s.search != SEARCH_UNDO)
            return NEXT;
        if (r.now - s.search_since < s.search_turned_ms)
        {
            search_pivot(FORK_SIDE, -1);
            return HANDLED;
        }
        s.search = SEARCH_NONE;
        s.search_done = true;
        s.lost = false; // si sigue sin linea, la perdida cuenta desde ahora
        return NEXT;
    }

    // Con el central sobre la linea se puede volver a buscar la rama de FORK_SIDE.
    Result forget_search(Reading &r)
    {
        if (r.black[SENSOR_CENTER])
            state.search_done = false;
        return NEXT;
    }

    // Respaldo por tiempo: los 3 en negro durante FINISH_MS es la llegada; un cruce se
    // atraviesa antes.
    Result track_all_black(Reading &r)
    {
        State &s = state;
        if (!(both_sides_black(r) && r.black[SENSOR_CENTER]))
        {
            // Salio del negro antes de FINISH_MS: era un cruce.
            s.in_all_black = s.in_all_black && r.now - s.all_black_last <= ALL_BLACK_TOLERANCE_MS;
            return NEXT;
        }
        if (!s.in_all_black)
            s.all_black_start = r.now;
        s.in_all_black = true;
        mark_all_black(r.now);
        return r.now - s.all_black_start >= FINISH_MS ? finish() : NEXT;
    }

    // Linea horizontal (salida, cruce o llegada): recto a velocidad fija. Basta con los dos
    // laterales: a velocidad, el central puede no coincidir con ellos en la misma lectura
    // y la llegada pasaria sin detectarse.
    Result cross_horizontal(Reading &r)
    {
        if (!both_sides_black(r))
            return NEXT;
        mark_all_black(r.now);
        stop_pid();
        state.lost = false;
        drive_on_line(ALL_BLACK_SPEED, ALL_BLACK_SPEED);
        return HANDLED;
    }

    // Solo el lateral contrario a FORK_SIDE ve la linea: antes de girar hacia ese lado
    // busca una rama hacia FORK_SIDE (en una Y puede quedar demasiado lejos para verla).
    Result search_before_other_side(Reading &r)
    {
        const bool only_other = r.black[OTHER_SENSOR] && !r.black[SENSOR_CENTER] && !r.black[SIDE_SENSOR];
        if (!(PRIORITIZE_FORK_SIDE && !state.search_done && only_other))
            return NEXT;
        stop_pid();
        start_search(r.now);
        return HANDLED;
    }

    // Recien salido de una linea horizontal: sigue recto; si la linea no aparece delante
    // en FINISH_CHECK_MS, era la llegada.
    Result check_finish_line(Reading &r)
    {
        if (r.now - state.lost_since < FINISH_CHECK_MS)
        {
            drive(ALL_BLACK_SPEED, ALL_BLACK_SPEED);
            return HANDLED;
        }
        back_to_finish();
        return finish();
    }

    // Movimiento con la linea perdida, segun cuanto hace y como se perdio.
    void drive_lost(unsigned long lost_ms)
    {
        const State &s = state;
        if (s.lost_centered && lost_ms < FORK_SECOND_MS)
            search_pivot(-FORK_SIDE, 1); // ya busco hacia FORK_SIDE: prueba el otro lado
        else if (!s.lost_centered && lost_ms < LOST_GRACE_MS)
            drive(s.last_speed_l, s.last_speed_r);
        else
            // Retrocede. Hacia atras, el morro gira hacia el lado cuya rueda retrocede mas
            // rapido; se usa para volver hacia donde la vio por ultima vez.
            drive(-RECOVER_SPEED + s.last_side * RECOVER_TURN,
                  -RECOVER_SPEED - s.last_side * RECOVER_TURN);
    }

    // Ningun sensor ve la linea.
    Result recover_line(Reading &r)
    {
        State &s = state;
        stop_pid();
        if (!s.lost)
        {
            s.lost = true;
            s.lost_since = r.now;
            s.lost_centered = s.centered;
        }
        if (s.seen_all_black && s.lost_since - s.all_black_last <= AFTER_BLACK_MS)
            return check_finish_line(r);

        // Venia centrado (hueco de una Y, o la linea quedo entre dos sensores): busca
        // primero hacia FORK_SIDE.
        if (s.lost_centered && !s.search_done)
            start_search(r.now);
        else
            drive_lost(r.now - s.lost_since);
        return HANDLED;
    }

    // PID con la linea a la vista.
    void follow_pid(int error)
    {
        State &s = state;
        s.lost = false;
        s.centered = error == 0;
        if (error != 0)
            s.last_side = error < 0 ? -1 : 1;

        // Compute() solo actualiza pid_output cuando ha pasado el sample time.
        pid_input = error;
        pid.SetMode(AUTOMATIC);
        pid.Compute();

        // speeds.base hasta SLOWDOWN_FROM_ERROR; de ahi a MAX_ERROR (solo un lateral) baja
        // hasta speeds.curve.
        const long abs_error = min(abs(error), MAX_ERROR);
        const long over = max(abs_error - SLOWDOWN_FROM_ERROR, 0L);
        const int base = speeds.base - (long)(speeds.base - speeds.curve) * over / (MAX_ERROR - SLOWDOWN_FROM_ERROR);

        // Con REVERSE y setpoint 0, pid_output tiene el signo del error: error > 0 (linea a
        // la derecha) acelera la rueda izquierda y frena la derecha, y gira a la derecha.
        // En curvas cerradas la rueda interior baja hasta speeds.inner_min.
        const int correction = static_cast<int>(pid_output);
        drive_on_line(max(base + correction, speeds.inner_min), max(base - correction, speeds.inner_min));
    }

    // Prioridad FORK_SIDE > recto > el otro lado: mientras el central vea la linea, el
    // lateral contrario a FORK_SIDE no cuenta para el PID.
    Result follow_line(Reading &r)
    {
        if (PRIORITIZE_FORK_SIDE && r.black[SENSOR_CENTER])
            r.black[OTHER_SENSOR] = false;

        int error = 0;
        if (!LineSensors<3>::line_error(r.values, r.black, error))
            return recover_line(r);
        follow_pid(error);
        return HANDLED;
    }

    typedef Result (*Phase)(Reading &);
    // En orden: la primera que no devuelve NEXT decide la vuelta. follow_line siempre decide.
    const Phase PHASES[] = {
        ignore_other_branch,
        leave_start,
        search_side,
        undo_search,
        forget_search,
        track_all_black,
        cross_horizontal,
        search_before_other_side,
        follow_line,
    };

    // Una vuelta del seguidor. Devuelve false al llegar a la meta.
    bool step()
    {
        Reading r;
        r.now = millis();
        hw->sensors.read(r.values, r.black);

        Result result = NEXT;
        for (size_t i = 0; result == NEXT && i < sizeof(PHASES) / sizeof(PHASES[0]); i++)
            result = PHASES[i](r);
        return result != FINISHED;
    }

    void task(void *)
    {
        const TickType_t period = pdMS_TO_TICKS(PERIOD_MS) > 0 ? pdMS_TO_TICKS(PERIOD_MS) : 1;

        // La correccion puede llegar a MAX_SPEED: en curvas cerradas la rueda interior va hacia atras.
        pid.SetOutputLimits(-Movement::MAX_SPEED, Movement::MAX_SPEED);
        pid.SetSampleTime(PERIOD_MS - 5); // < period para que Compute() calcule a tiempo
        pid.SetMode(AUTOMATIC);
        state.start_black_last = millis();

        TickType_t last_wake = xTaskGetTickCount();
        // Periodo fijo: el dt del PID es siempre el mismo y la tarea se bloquea en cada
        // vuelta, dejando correr a las demas.
        while (step())
            xTaskDelayUntil(&last_wake, period);

        hw->wheels.stop();
        hw->buzzer.beep_repeat(2, 800, 50);
        handle = nullptr;
        vTaskDelete(nullptr);
    }

    // Empieza un recorrido desde la linea de salida. Devuelve false si ya habia uno en
    // marcha o no hay memoria para la tarea.
    bool start(Components &components, configSTACK_DEPTH_TYPE stack)
    {
        if (handle != nullptr)
            return false;
        hw = &components;
        state = State();
        return xTaskCreate(task, "follow_line", stack, nullptr, PRIORITY, &handle) == pdPASS;
    }

    // Detiene el recorrido en marcha. Solo desde una tarea de menor prioridad: asi el
    // seguidor esta esperando su siguiente vuelta y no se corta a mitad de un paso.
    bool stop()
    {
        if (handle == nullptr)
            return false;
        vTaskDelete(handle);
        handle = nullptr;
        hw->wheels.stop();
        return true;
    }

    bool running()
    {
        return handle != nullptr;
    }
}

#endif
