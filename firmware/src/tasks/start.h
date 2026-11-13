#ifndef START_TASK_H
#define START_TASK_H

#include "robot.h"
#include "line_follower.h"

// Tarea de control "Start": lleva el robot de la zona de inicio a la meta.
//
// El ciclo esta en Robot::run(), al final del archivo: es una lista de pasos, y cada
// paso es una funcion de este archivo. El seguimiento de la linea en si (PID, serpenteo
// y busqueda) esta en line_follower.h.

// --- Parametros del recorrido ---

// El tick de FreeRTOS es de unos 16 ms: pdMS_TO_TICKS(30) daria 1 solo tick.
const TickType_t LOOP_TICKS = 2;
// Tiempo maximo para salir de la zona de inicio y cruzar su linea horizontal.
const unsigned long START_ZONE_MS = 3000;
// En una Y abierta los sensores laterales apenas coinciden sobre las ramas: basta con
// que cada uno vea linea dentro de esta ventana.
const unsigned long SIDE_SEEN_MS = 80;
// Giro minimo al tomar una rama, por si el sensor central sigue en el vertice.
const uint32_t FORK_TURN_MIN_MS = 100;
// Tiempo sin buscar otra bifurcacion despues de salir de una.
const unsigned long FORK_WAIT_MS = 1000;
// Si encontrar la linea tarda esto o mas, el giro fue de media vuelta: la linea se
// acabo y el robot la recupero detras. Unos 110 grados.
const unsigned long NO_EXIT_TURN_MS = 600;
// Giro minimo al volver a una Y, para dejar la rama de la que viene.
const uint32_t BACK_TURN_MIN_MS = 200;
// Obstaculos: distancia de deteccion, muestras seguidas para confirmarla y cada
// cuantos ciclos (~160 ms) se mide.
const long OBSTACLE_DISTANCE_CM = 15;
const uint8_t OBSTACLE_CHECKS = 2;
const uint8_t OBSTACLE_EVERY_LOOPS = 5;

// --- Estado de un recorrido ---

// Todo lo que la tarea de control recuerda de un ciclo al siguiente. Vive en la pila
// de la tarea: un recorrido nuevo empieza con un estado nuevo.
struct RunState
{
    // PID, serpenteo y busqueda de la linea.
    LineFollower follower;

    // Zona de inicio.
    unsigned long start_time;
    bool in_start_zone = true;
    bool start_line_passed = false;

    // Bifurcaciones: ultima vez que cada sensor lateral vio linea, y pausa despues de
    // salir de una bifurcacion.
    unsigned long left_seen_time;
    unsigned long right_seen_time;
    bool fork_wait = false;
    unsigned long fork_time = 0;

    // Linea perdida.
    bool line_lost = false;
    unsigned long line_lost_time = 0;

    // Callejon sin salida: de regreso hacia la ultima bifurcacion.
    bool going_back = false;
    unsigned long branch_start_time;
    unsigned long back_start_time = 0;
    unsigned long back_time_ms = 0;

    // Obstaculos.
    uint8_t obstacle_loop = 0;
    uint8_t obstacle_count = 0;

    RunState(float kp, float ki, float kd)
        : follower(kp, ki, kd),
          start_time(millis()),
          left_seen_time(millis() - SIDE_SEEN_MS),
          right_seen_time(left_seen_time),
          branch_start_time(millis()) {}
};

// --- Maniobras ---

// Cruza en linea recta cuando los 3 sensores ven linea: puede ser la meta, un cruce o
// el vertice de una Y, que deja los sensores laterales sobre las ramas y el central fuera.
template <size_t N>
CrossResult Robot<N>::cross_horizontal_line()
{
    const unsigned long max_crossing_ms = 1000;
    const unsigned long clear_line_ms = 120;

    const unsigned long crossing_start = millis();
    movement.drive(speed_settings.base, speed_settings.base);
    while (line_sensors.all_detected())
    {
        if (millis() - crossing_start > max_crossing_ms)
        {
            // Ninguna linea de meta es tan ancha: p. ej. el robot esta levantado.
            movement.stop();
            note_stop("invalid_reading");
            while (line_sensors.all_detected())
                vTaskDelay(1);
            return CROSS_CONTINUE;
        }
        vTaskDelay(1);
    }

    // Avanza un poco mas para que una llegada en diagonal no parezca una linea que continua.
    const unsigned long clear_start = millis();
    while (millis() - clear_start < clear_line_ms)
    {
        if (line_sensors.intersection_detected())
            return CROSS_FORK;
        vTaskDelay(1);
    }

    int error = 0;
    if (!line_sensors.read_line_error(error))
        return CROSS_FINISH;
    // Con linea solo a un lado era una Y muy abierta, no un cruce.
    return line_sensors.sensor_detected(N / 2) ? CROSS_CONTINUE : CROSS_FORK;
}

template <size_t N>
void Robot<N>::show_route(Route route)
{
    const bool left = route == ROUTE_LEFT;
    leds.turn_only(left ? 0 : 1);
    logger.event(left ? "route_left" : "route_right");
}

// Gira hasta que el sensor central encuentra una linea (ver tank_turn en robot.h).
template <size_t N>
void Robot<N>::turn_to(Route side, uint32_t min_turn_ms)
{
    // Con un lado detenido el giro es mas lento que sobre su eje.
    const unsigned long max_turn_ms = tank_turn ? 1500 : 2500;

    int speed_l;
    int speed_r;
    turn_speeds(side == ROUTE_LEFT ? -1 : 1, speed_l, speed_r);
    movement.drive(speed_l, speed_r);

    wait_ms(min_turn_ms);

    const unsigned long turn_start = millis();
    while (!line_sensors.sensor_detected(N / 2) && millis() - turn_start < max_turn_ms)
        vTaskDelay(1);
}

// Gira hacia una rama de la bifurcacion y empieza a recorrerla.
template <size_t N>
void Robot<N>::take_branch(RunState &state, Route side, uint32_t min_turn_ms)
{
    turn_to(side, min_turn_ms);
    // Si la linea se pierde al salir del giro, esta hacia el lado del giro.
    state.follower.search_to(side == ROUTE_LEFT ? -1 : 1);
    state.fork_wait = true;
    state.fork_time = millis();
    state.branch_start_time = state.fork_time;
}

// Anota por que el programa detuvo el robot y lo envia como evento. reason debe ser
// un literal: solo se guarda el puntero.
template <size_t N>
void Robot<N>::note_stop(const char *reason)
{
    // La tarea de telemetria lee estos dos valores: se cambian juntos.
    taskENTER_CRITICAL();
    last_stop_reason = reason;
    last_stop_ms = millis();
    taskEXIT_CRITICAL();
    logger.event(reason);
}

// Termina el recorrido desde la propia tarea de control. No vuelve.
template <size_t N>
void Robot<N>::end_run(const char *reason)
{
    movement.stop();
    leds.turn_off_all();
    note_stop(reason);
    // Sin tarea de control, una calibracion pedida despues empieza enseguida.
    control_task = nullptr;
    vTaskDelete(nullptr);
}

// --- Pasos del ciclo de control ---

// Cede el turno a la tarea de calibracion y espera a que termine.
template <size_t N>
void Robot<N>::wait_calibration(RunState &state)
{
    const unsigned long calibration_start = millis();
    movement.stop();
    calibration_requested = false;
    xTaskNotifyGive(calibration_task);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // La calibracion no cuenta como recorrido: al terminar sigue la linea.
    const unsigned long calibration_ms = millis() - calibration_start;
    state.branch_start_time += calibration_ms;
    state.back_start_time += calibration_ms;
    state.line_lost = false;
}

// Zona de inicio: el robot avanza recto hasta la linea y cruza la linea horizontal de
// inicio sin tomarla por la meta. Devuelve true mientras esta cruzando esa linea: en
// ese ciclo no hay nada mas que hacer.
template <size_t N>
bool Robot<N>::pass_start_line(RunState &state, bool line_detected)
{
    if (!state.in_start_zone)
        return false;

    const bool on_start_line = line_sensors.all_detected();
    if (millis() - state.start_time >= START_ZONE_MS ||
        (state.start_line_passed && line_detected && !on_start_line))
    {
        state.in_start_zone = false;
        return false;
    }
    if (!on_start_line)
        return false;

    state.start_line_passed = true;
    movement.drive(speed_settings.base, speed_settings.base);
    return true;
}

// Pasado FORK_WAIT_MS desde la ultima bifurcacion, vuelve a buscar bifurcaciones.
template <size_t N>
void Robot<N>::end_fork_wait(RunState &state)
{
    if (state.fork_wait && millis() - state.fork_time >= FORK_WAIT_MS)
    {
        leds.turn_off_all();
        state.fork_wait = false;
    }
}

// Anota cuando se pierde la linea y cuando se recupera. Si recuperarla llevo mucho
// giro, el robot dio media vuelta en un callejon sin salida y empieza el regreso.
template <size_t N>
void Robot<N>::check_line_lost(RunState &state, bool line_detected)
{
    if (!line_detected && !state.line_lost)
    {
        state.line_lost = true;
        state.line_lost_time = millis();
        return;
    }
    if (!line_detected || !state.line_lost)
        return;

    state.line_lost = false;
    if (!state.going_back && route_memory.depth() > 0 &&
        millis() - state.line_lost_time >= NO_EXIT_TURN_MS)
    {
        state.going_back = true;
        state.back_start_time = millis();
        state.back_time_ms = state.line_lost_time - state.branch_start_time;
        leds.turn_off_all();
        logger.event("dead_end");
    }
}

// Devuelve true si los dos sensores laterales vieron linea casi a la vez: el robot
// esta sobre el vertice de una Y.
template <size_t N>
bool Robot<N>::see_fork(RunState &state)
{
    const unsigned long now = millis();
    if (!state.line_lost && line_sensors.sensor_detected(0))
        state.left_seen_time = now;
    if (!state.line_lost && line_sensors.sensor_detected(N - 1))
        state.right_seen_time = now;

    return !state.in_start_zone && !state.fork_wait && !state.line_lost &&
           now - state.left_seen_time < SIDE_SEEN_MS &&
           now - state.right_seen_time < SIDE_SEEN_MS;
}

// Recorrido normal: cruza las lineas horizontales (la meta termina el recorrido) y
// toma la rama preferida en las bifurcaciones. Devuelve true si hizo una maniobra:
// el ciclo vuelve a empezar con lecturas nuevas.
template <size_t N>
bool Robot<N>::follow_route(RunState &state, bool fork_seen)
{
    bool at_fork = fork_seen;

    if (!state.fork_wait && line_sensors.all_detected())
    {
        const CrossResult result = cross_horizontal_line();
        if (result == CROSS_FINISH)
            end_run("finish");
        if (result == CROSS_CONTINUE)
            return true;
        at_fork = true;
    }

    if (!at_fork)
        return false;

    if (!route_memory.push(preferred_route, millis() - state.branch_start_time))
        end_run("memory_full");

    show_route(preferred_route);
    take_branch(state, preferred_route, FORK_TURN_MIN_MS);
    return true;
}

// Regreso de un callejon sin salida: al llegar otra vez a la bifurcacion toma la otra
// rama o, si ya probo las dos, sigue hacia la bifurcacion anterior. Devuelve true si
// giro en la bifurcacion: el ciclo vuelve a empezar con lecturas nuevas.
template <size_t N>
bool Robot<N>::go_back_to_fork(RunState &state, bool fork_seen)
{
    const unsigned long back_time = millis() - state.back_start_time;
    if (back_time > state.back_time_ms + state.back_time_ms / 2 + 500)
        end_run("fork_missed");

    // De vuelta, los brazos de la Y quedan a ambos lados del robot.
    if (!fork_seen || back_time < state.back_time_ms / 2)
        return false;

    const ReturnDecision decision = route_memory.on_return();
    if (decision.action == ReturnDecision::NO_ROUTE_LEFT)
        end_run("no_route");

    if (decision.action == ReturnDecision::TAKE_OTHER_BRANCH)
    {
        state.going_back = false;
        show_route(decision.other_branch);
    }
    else
    {
        leds.turn_off_all();
        logger.event("back_to_stem");
    }

    take_branch(state, decision.turn, BACK_TURN_MIN_MS);
    state.back_start_time = state.fork_time;
    state.back_time_ms = decision.stem_ms;
    return true;
}

// Mide la distancia cada OBSTACLE_EVERY_LOOPS ciclos y retira el objeto si hay uno
// delante. Devuelve true si lo retiro: el ciclo vuelve a empezar con lecturas nuevas.
// Desactivada por ahora (ver Robot::run): daba falsos positivos.
template <size_t N>
bool Robot<N>::remove_obstacle(RunState &state, bool line_detected)
{
    state.obstacle_loop = state.obstacle_loop + 1 >= OBSTACLE_EVERY_LOOPS ? 0 : state.obstacle_loop + 1;
    if (state.going_back || !line_detected || state.obstacle_loop != 0)
        return false;

    const long distance = obstacle_sensor.read_distance();
    state.obstacle_count = distance < OBSTACLE_DISTANCE_CM ? state.obstacle_count + 1 : 0;
    if (state.obstacle_count < OBSTACLE_CHECKS)
        return false;

    state.obstacle_count = 0;
    // El retiro no cuenta como recorrido de la rama.
    const unsigned long removal_start = millis();
    remove_object();
    state.branch_start_time += millis() - removal_start;
    // El giro de vuelta se queda corto: la linea queda a la izquierda.
    state.follower.search_to(-1);
    return true;
}

// Publica la muestra del ciclo: la tarea de telemetria toma la mas reciente cuando le
// toca enviar, sin frenar el control.
template <size_t N>
void Robot<N>::send_sample(int line_error, bool line_detected, const WheelSpeeds &speeds)
{
    if (!logger.is_recording())
        return;

    TelemetrySample sample;
    sample.ms = millis();
    for (size_t index = 0; index < N; ++index)
        sample.raw[index] = line_sensors.raw_read(index);
    sample.error = line_error;
    sample.correction = speeds.correction;
    sample.speed_l = speeds.left;
    sample.speed_r = speeds.right;
    sample.line_detected = line_detected;
    xQueueOverwrite(telemetry_samples, &sample);
}

// --- Tarea de control ---

// Cuerpo de la tarea "Start": un ciclo cada LOOP_TICKS (~32 ms). No vuelve:
// el recorrido termina con end_run() (meta o error) o cuando stop() borra la tarea.
template <size_t N>
void Robot<N>::run()
{
    // Ganancias guardadas en EEPROM o las de robot.h (ver pid_gains.h).
    float kp;
    float ki;
    float kd;
    read_gains(kp, ki, kd);
    RunState state(kp, ki, kd);

    while (1)
    {
        // 1. Pedidos del PC: ganancias nuevas y calibracion.
        if (gains_changed)
        {
            read_gains(kp, ki, kd);
            state.follower.set_gains(kp, ki, kd);
        }
        if (calibration_requested)
        {
            wait_calibration(state);
            continue;
        }

        // 2. Donde esta la linea.
        int line_error = 0;
        const bool line_detected = line_sensors.read_line_error(line_error);

        // 3. Zona de inicio: cruza recto la linea horizontal de salida.
        if (pass_start_line(state, line_detected))
        {
            vTaskDelay(LOOP_TICKS);
            continue;
        }

        // 4. Linea perdida y recuperada: detecta los callejones sin salida.
        end_fork_wait(state);
        check_line_lost(state, line_detected);

        // 5. Bifurcaciones, lineas horizontales y meta. Despues de una maniobra el
        //    ciclo vuelve a empezar con lecturas nuevas.
        const bool fork_seen = see_fork(state);
        const bool moved = state.going_back ? go_back_to_fork(state, fork_seen)
                                          : follow_route(state, fork_seen);
        if (moved)
            continue;

        // 6. Obstaculos. Desactivado por ahora: daba falsos positivos.
        // if (remove_obstacle(state, line_detected))
        //     continue;

        // 7. Seguimiento: PID sobre el error, serpenteo si la linea esta centrada y
        //    giro de busqueda si se perdio (ver line_follower.h).
        const WheelSpeeds speeds = state.follower.get_speeds(line_detected, line_error, state.in_start_zone);
        movement.drive(speeds.left, speeds.right);

        send_sample(line_error, line_detected, speeds);
        vTaskDelay(LOOP_TICKS);
    }
}

// Empieza un recorrido: crea la tarea de control "Start". Se llama en setup(), para
// que el robot arranque al encender, y desde la tarea de telemetria cuando el PC pide
// otro recorrido. Devuelve false si ya hay un recorrido en marcha,
// si hay una calibracion pendiente o si no queda memoria para la tarea.
template <size_t N>
bool Robot<N>::start()
{
    if (control_task != nullptr || calibration_requested || calibration_state != CALIBRATION_IDLE)
        return false;

    // Un recorrido nuevo no hereda las bifurcaciones del anterior.
    route_memory.clear();
    last_stop_reason = nullptr;
    leds.turn_off_all();
    return xTaskCreate([](void *parameter)
                       { static_cast<Robot<N> *>(parameter)->run(); },
                       // En AVR el tamano del stack esta en bytes y el estado del
                       // recorrido, con el PID, vive en el.
                       "Start", 768, this, 2, &control_task) == pdPASS;
}

// Detiene el recorrido en marcha desde otra tarea: borra la tarea de control y para los
// motores. La llama la tarea de telemetria cuando el PC lo pide. Sin recorrido en marcha
// solo para los motores. Despues, start() empieza un recorrido nuevo desde cero.
template <size_t N>
void Robot<N>::stop()
{
    // Con el planificador detenido, la tarea de control no puede terminar por su cuenta
    // (end_run) entre la comprobacion y el borrado.
    vTaskSuspendAll();
    if (control_task != nullptr)
    {
        vTaskDelete(control_task);
        control_task = nullptr;
    }
    xTaskResumeAll();

    movement.stop();
    leds.turn_off_all();
}

#endif
