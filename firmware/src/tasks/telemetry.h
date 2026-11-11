#ifndef TELEMETRY_TASK_H
#define TELEMETRY_TASK_H

#include "robot.h"

// Cabecera de cada registro: umbrales en uso (los de la EEPROM si hay calibracion
// guardada), ganancias del PID y nombres de las columnas de telemetria.
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
    log_gains();
    logger.println("t,ms,s0,s1,s2,error,correction,left,right,line");
}

template <size_t N>
void Robot<N>::log_calibration_report()
{
    logger.println("calibration,sensor,min,max,threshold");
    for (size_t index = 0; index < N; ++index)
    {
        logger.print("calibration,");
        logger.print(index);
        logger.print(',');
        logger.print(calibration_minimums[index]);
        logger.print(',');
        logger.print(calibration_maximums[index]);
        logger.print(',');
        logger.println(line_sensors.threshold(index));
    }
    logger.println("calibration,complete");
}

// Pide una calibracion. Si el robot esta en recorrido, la tarea de control se detiene
// entre dos ciclos y cede el turno; si no, la tarea de calibracion empieza enseguida.
template <size_t N>
void Robot<N>::request_calibration()
{
    if (calibration_requested || calibration_state != CALIBRATION_IDLE)
        return;

    if (control_task != nullptr)
        calibration_requested = true;
    else
        xTaskNotifyGive(calibration_task);
}

// Crea las tareas de servicio. Se llama desde setup(), antes de que arranque FreeRTOS.
//
// - "Telemetry" (prioridad 1): la unica que usa el puerto serie. Lee los comandos del
//   PC y envia la telemetria, los eventos y el resultado de la calibracion. Con menos
//   prioridad que el control, un puerto lento nunca retrasa el seguimiento de linea.
//   Tambien crea la tarea de control ("Start", start.h) cuando el PC pide otro recorrido.
// - "Calibration" (prioridad 2): duerme hasta que le piden calibrar.
template <size_t N>
void Robot<N>::start_services()
{
    telemetry_samples = xQueueCreate(1, sizeof(TelemetrySample));

    xTaskCreate([](void *parameter)
                {
                    Robot<N> *robot = static_cast<Robot<N> *>(parameter);
                    // Unos 100 ms: 10 filas por segundo. Una fila ocupa unos 45 bytes y a
                    // 9600 baudios caben unas 20 por segundo.
                    const TickType_t telemetry_period_ticks = 6;
                    bool calibration_announced = false;

                    while (1)
                    {
                        const uint8_t requests = robot->logger.poll_commands();
                        if (requests & Logger::REQUEST_HEADER)
                            robot->log_header();
                        if (requests & Logger::REQUEST_CALIBRATION)
                            robot->request_calibration();
                        if (requests & Logger::REQUEST_GAINS_SET)
                        {
                            const float *gains = robot->logger.gain_values();
                            if (!robot->set_gains(gains[0], gains[1], gains[2]))
                                robot->logger.println("pid,error");
                        }
                        if (requests & Logger::REQUEST_GAINS_INVALID)
                            robot->logger.println("pid,error");
                        // Formato CSV: pid,<kp>,<ki>,<kd> con las ganancias en uso, o pid,error
                        if (requests & (Logger::REQUEST_GAINS_SET | Logger::REQUEST_GAINS_QUERY))
                            robot->log_gains();

                        // Antes que REQUEST_RUN: "xg" detiene el recorrido y empieza otro.
                        if (requests & Logger::REQUEST_HALT)
                        {
                            // Formato CSV: run,<ms>,stop
                            robot->stop();
                            robot->logger.print("run,");
                            robot->logger.print(millis());
                            robot->logger.println(",stop");
                        }
                        if (requests & Logger::REQUEST_RUN)
                        {
                            // Formato CSV: run,<ms>,start o run,<ms>,ignored
                            const bool started = robot->start();
                            robot->logger.print("run,");
                            robot->logger.print(millis());
                            robot->logger.println(started ? ",start" : ",ignored");
                        }
                        // La tarea de control termino sin atender la peticion: p. ej. llego a la meta.
                        if (robot->calibration_requested && robot->control_task == nullptr)
                        {
                            robot->calibration_requested = false;
                            xTaskNotifyGive(robot->calibration_task);
                        }

                        if (robot->calibration_state == CALIBRATION_RUNNING && !calibration_announced)
                        {
                            robot->logger.println("calibration,start");
                            calibration_announced = true;
                        }
                        else if (robot->calibration_state == CALIBRATION_DONE)
                        {
                            robot->log_calibration_report();
                            calibration_announced = false;
                            robot->calibration_state = CALIBRATION_IDLE;
                        }

                        robot->logger.flush_events();

                        TelemetrySample sample;
                        if (xQueueReceive(robot->telemetry_samples, &sample, 0) == pdTRUE)
                        {
                            int row[N + 5];
                            for (size_t index = 0; index < N; ++index)
                                row[index] = sample.raw[index];
                            row[N] = sample.error;
                            row[N + 1] = sample.correction;
                            row[N + 2] = sample.speed_l;
                            row[N + 3] = sample.speed_r;
                            row[N + 4] = sample.line_detected;
                            robot->logger.telemetry(sample.ms, row, N + 5);
                        }

                        vTaskDelay(telemetry_period_ticks);
                    } },
                // Mas pila que antes: convierte y escribe decimales y guarda en EEPROM.
                "Telemetry", 448, this, 1, nullptr);

    xTaskCreate([](void *parameter)
                {
                    Robot<N> *robot = static_cast<Robot<N> *>(parameter);

                    while (1)
                    {
                        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
                        robot->run_calibration();

                        // Devuelve el turno a la tarea de control, que esperaba detenida.
                        if (robot->control_task != nullptr)
                            xTaskNotifyGive(robot->control_task);
                    } },
                "Calibration", 256, this, 2, &calibration_task);
}

#endif
