#ifndef START_TASK_H
#define START_TASK_H

#include <PID_v1.h>
#include "robot.h"

// Cruza en linea recta cuando los 3 sensores ven linea: puede ser la meta, un cruce o
// el vertice de una Y, que deja los sensores laterales sobre las ramas y el central fuera.
template <size_t N>
CrossResult Robot<N>::cross_horizontal_line()
{
    const unsigned long max_crossing_ms = 1000;
    const unsigned long clear_line_ms = 120;

    const unsigned long crossing_start = millis();
    movement.drive(BASE_SPEED, BASE_SPEED);
    while (line_sensors.all_detected())
    {
        if (millis() - crossing_start > max_crossing_ms)
        {
            // Ninguna linea de meta es tan ancha: p. ej. el robot esta levantado.
            movement.stop();
            logger.event("invalid_reading");
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

// Cabecera de cada registro: umbrales en uso (los de la EEPROM si hay calibracion
// guardada) y nombres de las columnas de telemetria.
template <size_t N>
void Robot<N>::log_header()
{
    logger.print("thresholds");
    for (size_t index = 0; index < N; ++index)
    {
        logger.print(',');
        logger.print(line_sensors.threshold(index));
    }
    logger.println("");
    logger.println("t,ms,s0,s1,s2,error,correction,left,right,line");
}

template <size_t N>
void Robot<N>::show_route(Route route)
{
    const bool left = route == ROUTE_LEFT;
    leds.turn_only(left ? 0 : 1);
    logger.event(left ? "route_left" : "route_right");
}

// Gira sobre su eje hasta que el sensor central encuentra una linea.
template <size_t N>
void Robot<N>::turn_toward(Route side, uint32_t min_turn_ms)
{
    const unsigned long max_turn_ms = 1500;

    if (side == ROUTE_LEFT)
        movement.rotate_left(TURN_SPEED);
    else
        movement.rotate_right(TURN_SPEED);

    wait_ms(min_turn_ms);

    const unsigned long turn_start = millis();
    while (!line_sensors.sensor_detected(N / 2) && millis() - turn_start < max_turn_ms)
        vTaskDelay(1);
}

template <size_t N>
void Robot<N>::halt(const char *reason)
{
    movement.stop();
    leds.turn_off_all();
    logger.event(reason);
    vTaskDelete(nullptr);
}

template <size_t N>
void Robot<N>::start(Route route)
{
    preferred_route = route;
    xTaskCreate([](void *parameter)
                {
                    Robot<N> *robot = static_cast<Robot<N> *>(parameter);
                    int search_direction = -1;
                    // El tick de FreeRTOS es de unos 16 ms: pdMS_TO_TICKS(30) daria 1 solo tick.
                    const TickType_t control_period_ticks = 2;
                    // Menor que el periodo real (~32 ms) para que Compute() actue en cada ciclo.
                    const int pid_sample_time_ms = 25;
                    // Muestras seguidas para confirmar un obstaculo.
                    const uint8_t confirm_samples = 2;
                    // En una Y abierta los sensores laterales apenas coinciden sobre las ramas:
                    // basta con que cada uno vea linea dentro de esta ventana.
                    const unsigned long side_memory_ms = 80;
                    // Giro minimo al tomar una rama, por si el sensor central sigue en el vertice.
                    const uint32_t fork_turn_min_ms = 100;
                    // Tiempo sin buscar otra bifurcacion despues de salir de una.
                    const unsigned long fork_hold_ms = 1000;
                    const long obstacle_distance_cm = 15;
                    // Cada cuantos ciclos (~160 ms) se mide distancia y se envia telemetria.
                    const uint8_t slow_period_cycles = 5;
                    // Si encontrar la linea tarda esto o mas, el giro fue de media vuelta:
                    // la linea se acabo y el robot la recupero detras. Unos 110 grados.
                    const unsigned long dead_end_search_ms = 600;
                    // Giro minimo al volver a una Y, para dejar la rama de la que viene.
                    const uint32_t return_turn_min_ms = 200;
                    // Tiempo maximo para salir de la zona de inicio y cruzar su linea horizontal.
                    const unsigned long start_zone_ms = 3000;

                    const unsigned long task_started_at = millis();
                    bool start_zone_active = true;
                    bool start_line_crossed = false;

                    uint8_t cycle = 0;
                    uint8_t obstacle_samples = 0;
                    unsigned long left_seen_at = millis() - side_memory_ms;
                    unsigned long right_seen_at = left_seen_at;
                    bool fork_hold_active = false;
                    unsigned long fork_left_at = 0;

                    bool searching = false;
                    unsigned long line_lost_at = 0;
                    bool returning = false;
                    unsigned long branch_started_at = millis();
                    unsigned long return_started_at = 0;
                    unsigned long expected_return_ms = 0;

                    double line_error = 0;
                    double pid_output = 0;
                    double target = 0;
                    // Con error 500 (linea entre el centro y un lado) la correccion es 125:
                    // la rueda interior se detiene y la exterior va al maximo.
                    const double proportional_gain = 0.25;
                    const double integral_gain = 0.0;
                    // PID_v1 divide Kd entre el periodo en segundos (0.005 / 0.025 = 0.2 por muestra).
                    const double derivative_gain = 0.005;
                    PID line_pid(&line_error, &pid_output, &target,
                                 proportional_gain, integral_gain, derivative_gain, REVERSE);

                    line_pid.SetOutputLimits(-200, 200);
                    line_pid.SetSampleTime(pid_sample_time_ms);
                    line_pid.SetMode(AUTOMATIC);

                    // Cada cuantos ciclos (~100 ms) se envia una fila mientras se registra.
                    // Una fila ocupa unos 45 bytes: a 9600 baudios caben unas 20 por segundo.
                    const uint8_t telemetry_period_cycles = 3;
                    uint8_t telemetry_cycle = 0;

                    while(1){
                        if (robot->logger.poll_commands())
                            robot->log_header();

                        int current_error = 0;
                        const bool line_detected = robot->line_sensors.read_line_error(current_error);

                        if (start_zone_active)
                        {
                            const bool on_start_line = robot->line_sensors.all_detected();
                            if (millis() - task_started_at >= start_zone_ms ||
                                (start_line_crossed && line_detected && !on_start_line))
                            {
                                start_zone_active = false;
                            }
                            else if (on_start_line)
                            {
                                // La linea de inicio no es la meta: la cruza recto.
                                start_line_crossed = true;
                                robot->movement.drive(BASE_SPEED, BASE_SPEED);
                                vTaskDelay(control_period_ticks);
                                continue;
                            }
                        }

                        if (fork_hold_active && millis() - fork_left_at >= fork_hold_ms)
                        {
                            robot->leds.turn_off_all();
                            fork_hold_active = false;
                        }

                        if (!line_detected && !searching)
                        {
                            searching = true;
                            line_lost_at = millis();
                        }
                        else if (line_detected && searching)
                        {
                            searching = false;
                            if (!returning && robot->route_memory.depth() > 0 &&
                                millis() - line_lost_at >= dead_end_search_ms)
                            {
                                returning = true;
                                return_started_at = millis();
                                expected_return_ms = line_lost_at - branch_started_at;
                                robot->leds.turn_off_all();
                                robot->logger.event("dead_end");
                            }
                        }

                        const unsigned long now = millis();
                        if (!searching && robot->line_sensors.sensor_detected(0))
                            left_seen_at = now;
                        if (!searching && robot->line_sensors.sensor_detected(N - 1))
                            right_seen_at = now;
                        const bool both_sides_seen = !start_zone_active && !fork_hold_active && !searching &&
                                                     now - left_seen_at < side_memory_ms &&
                                                     now - right_seen_at < side_memory_ms;

                        if (returning)
                        {
                            const unsigned long return_elapsed = millis() - return_started_at;
                            if (return_elapsed > expected_return_ms + expected_return_ms / 2 + 500)
                                robot->halt("fork_missed");

                            // De vuelta, los brazos de la Y quedan a ambos lados del robot.
                            if (both_sides_seen && return_elapsed >= expected_return_ms / 2)
                            {
                                const ReturnDecision decision = robot->route_memory.on_return();
                                if (decision.action == ReturnDecision::NO_ROUTE_LEFT)
                                    robot->halt("no_route");

                                if (decision.action == ReturnDecision::TAKE_OTHER_BRANCH)
                                {
                                    returning = false;
                                    robot->show_route(decision.other_branch);
                                }
                                else
                                {
                                    robot->leds.turn_off_all();
                                    robot->logger.event("back_to_stem");
                                }

                                robot->turn_toward(decision.turn, return_turn_min_ms);
                                search_direction = decision.turn == ROUTE_LEFT ? -1 : 1;
                                fork_hold_active = true;
                                fork_left_at = millis();
                                branch_started_at = fork_left_at;
                                return_started_at = fork_left_at;
                                expected_return_ms = decision.stem_ms;
                                continue;
                            }
                        }
                        else
                        {
                            bool at_fork = both_sides_seen;

                            if (!fork_hold_active && robot->line_sensors.all_detected())
                            {
                                const CrossResult result = robot->cross_horizontal_line();
                                if (result == CROSS_FINISH)
                                    robot->halt("finish");
                                if (result == CROSS_CONTINUE)
                                    continue;
                                at_fork = true;
                            }

                            if (at_fork)
                            {
                                if (!robot->route_memory.push(robot->preferred_route, millis() - branch_started_at))
                                    robot->halt("memory_full");

                                robot->show_route(robot->preferred_route);
                                robot->turn_toward(robot->preferred_route, fork_turn_min_ms);
                                search_direction = robot->preferred_route == ROUTE_LEFT ? -1 : 1;
                                fork_hold_active = true;
                                fork_left_at = millis();
                                branch_started_at = fork_left_at;
                                continue;
                            }
                        }

                        cycle = cycle + 1 >= slow_period_cycles ? 0 : cycle + 1;
                        // Deteccion de colision desactivada por ahora: daba falsos positivos.
                        /*
                        if (!returning && line_detected && cycle == 0)
                        {
                            const long distance = robot->obstacle_sensor.read_distance();
                            obstacle_samples = distance < obstacle_distance_cm ? obstacle_samples + 1 : 0;
                            if (obstacle_samples >= confirm_samples)
                            {
                                obstacle_samples = 0;
                                // El retiro no cuenta como recorrido de la rama.
                                const unsigned long removal_start = millis();
                                robot->remove_object();
                                branch_started_at += millis() - removal_start;
                                // El giro de vuelta se queda corto: la linea queda a la izquierda.
                                search_direction = -1;
                                continue;
                            }
                        }
                        */

                        if (line_detected)
                        {
                            line_error = current_error;

                            if (current_error < 0)
                                search_direction = -1;
                            else if (current_error > 0)
                                search_direction = 1;
                        }

                        line_pid.Compute();
                        const int correction = static_cast<int>(pid_output);

                        int speed_l;
                        int speed_r;
                        if (line_detected)
                        {
                            speed_l = BASE_SPEED + correction;
                            speed_r = BASE_SPEED - correction;
                        }
                        else if (start_zone_active)
                        {
                            speed_l = BASE_SPEED;
                            speed_r = BASE_SPEED;
                        }
                        else
                        {
                            speed_l = search_direction < 0 ? -TURN_SPEED : TURN_SPEED;
                            speed_r = -speed_l;
                        }
                        robot->movement.drive(speed_l, speed_r);

                        telemetry_cycle = telemetry_cycle + 1 >= telemetry_period_cycles ? 0 : telemetry_cycle + 1;
                        if (telemetry_cycle == 0 && robot->logger.is_recording())
                        {
                            int row[N + 5];
                            for (size_t index = 0; index < N; ++index)
                                row[index] = robot->line_sensors.raw_read(index);
                            row[N] = current_error;
                            row[N + 1] = correction;
                            row[N + 2] = speed_l;
                            row[N + 3] = speed_r;
                            row[N + 4] = line_detected;
                            robot->logger.telemetry(row, N + 5);
                        }

                        vTaskDelay(control_period_ticks);
                    } },
                // En AVR el tamano del stack esta en bytes y el objeto PID vive en el.
                "Start", 768, this, 2, nullptr);
}

#endif
