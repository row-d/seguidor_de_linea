#ifndef START_TASK_H
#define START_TASK_H

#include <PID_v1.h>
#include "robot.h"

template <size_t N>
void Robot<N>::start()
{
    xTaskCreate([](void *parameter)
                {
                    Robot<N> *robot = static_cast<Robot<N> *>(parameter);
                    int previous_direction = -1;
                    int worsening_samples = 0;
                    int led_direction = 0;
                    const int base_speed = 90;
                    const int control_period_ms = 30;
                    
                    double line_error = 0;
                    double pid_output = 0;
                    double target = 0;
                    int last_error = 0;
                    const double proportional_gain = 0.30;
                    const double integral_gain = 0.0;
                    const double derivative_gain = 0.80;
                    PID line_pid(&line_error, &pid_output, &target,
                                 proportional_gain, integral_gain, derivative_gain, REVERSE);

                    line_pid.SetOutputLimits(-150, 150);
                    line_pid.SetSampleTime(control_period_ms);
                    line_pid.SetMode(AUTOMATIC);

                    while(1){
                        int current_error = 0;
                        const bool line_detected = robot->line_sensors.read_line_error(current_error);

                        if (robot->line_sensors.intersection_detected())
                        {
                            robot->leds.turn_only(0);
                        }

                        if (line_detected)
                        {
                            line_error = current_error;
                            last_error = current_error;
                            worsening_samples = 0;

                            if (current_error < 0)
                                previous_direction = -1;
                            else if (current_error > 0)
                                previous_direction = 1;
                        }

                        line_pid.Compute();
                        const int correction = constrain(static_cast<int>(pid_output), -120, 120);

                        if (correction < -25)
                        {
                            led_direction = -1;
                        }
                        else if (correction > 25)
                        {
                            led_direction = 1;
                        }
                        else if (abs(correction) < 8)
                        {
                            led_direction = 0;
                        }

                        if (led_direction < 0)
                        {
                            robot->leds.turn_only(0);
                        }
                        else if (led_direction > 0)
                        {
                            robot->leds.turn_only(1);
                        }
                        else
                        {
                            robot->leds.turn_off_all();
                        }

                        if (line_detected)
                        {
                            robot->movement.move_forward(
                                base_speed + correction,
                                base_speed - correction);
                        }
                        else
                        {
                            ++worsening_samples;
                            if (worsening_samples >= 4)
                            {
                                previous_direction = -previous_direction;
                                worsening_samples = 0;
                            }

                            const int recovery_error = last_error == 0 ? previous_direction : last_error;
                            robot->movement.move_forward(
                                base_speed + (recovery_error < 0 ? -100 : 100),
                                base_speed + (recovery_error < 0 ? 100 : -100));
                        }

                        vTaskDelay(pdMS_TO_TICKS(control_period_ms));
                    } },
                "Start", 192, this, 2, nullptr);
}

#endif
