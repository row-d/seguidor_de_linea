#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include "component.h"
#include "movement.h"
#include "line_sensors.h"
#include "color_sensor.h"
#include "claw.h"
#include "buzzer.h"
#include "led_array.h"
#include "ultrasonic_sensor.h"

template <size_t N>
class Robot : public Component
{
public:
private:
    Movement &movement;
    LineSensors<N> &line_sensors;
    ColorSensor &color_sensor;
    Claw &claw;
    Buzzer &buzzer;
    LedArray<2> &turn_leds;
    UltrasonicSensor &obstacle_sensor;

public:
    Robot(Movement &movement_controller,
          LineSensors<N> &sensors,
          ColorSensor &color,
          Claw &robot_claw,
          Buzzer &robot_buzzer,
          LedArray<2> &robot_turn_leds,
          UltrasonicSensor &robot_obstacle_sensor)
        : movement(movement_controller),
          line_sensors(sensors),
          color_sensor(color),
          claw(robot_claw),
          buzzer(robot_buzzer),
          turn_leds(robot_turn_leds),
          obstacle_sensor(robot_obstacle_sensor) {}

    void setup()
    {
        movement.setup();
        line_sensors.setup();
        color_sensor.setup();
        claw.setup();
        buzzer.setup();
        turn_leds.setup();
        obstacle_sensor.setup();
    }

    void start()
    {

        xTaskCreate([](void *parameter)
                    {
                        Robot<N> *robot = static_cast<Robot<N> *>(parameter);
                        bool obstacle_handled = false;
                        int previous_error = 0;
                        long integral = 0;
                        const int base_speed = 170;
                        const float proportional_gain = 0.12f;
                        const float integral_gain = 0.002f;
                        const float derivative_gain = 0.35f;
                        while (1)
                        {
                            const bool obstacle_detected = robot->obstacle_sensor.read_distance() < 15;
                            if (obstacle_detected && !obstacle_handled)
                            {
                                obstacle_handled = true;
                                robot->movement.stop();
                                robot->claw.open();
                                vTaskDelay(pdMS_TO_TICKS(150));
                                robot->movement.move_forward(140);
                                vTaskDelay(pdMS_TO_TICKS(350));
                                robot->movement.stop();
                                robot->claw.close();
                                vTaskDelay(pdMS_TO_TICKS(400));
                                robot->movement.move_backward(140);
                                vTaskDelay(pdMS_TO_TICKS(500));
                                robot->movement.stop();
                                robot->claw.open();
                                vTaskDelay(pdMS_TO_TICKS(250));
                                robot->buzzer.beep(120);
                            }
                            else if (obstacle_detected)
                            {
                                robot->movement.stop();
                            }
                            else
                            {
                                obstacle_handled = false;
                                int error = previous_error;
                                const bool line_detected = robot->line_sensors.read_line_error(error);
                                const int derivative = error - previous_error;
                                if (line_detected)
                                {
                                    integral += error;
                                    integral = constrain(integral, -3000L, 3000L);
                                    previous_error = error;
                                }
                                else
                                {
                                    integral = 0;
                                }

                                const int correction = static_cast<int>(
                                    proportional_gain * error +
                                    integral_gain * integral +
                                    derivative_gain * derivative);
                                const int left_speed = constrain(base_speed + correction, 0, 255);
                                const int right_speed = constrain(-base_speed + correction, -255, 0);

                                robot->movement.move(left_speed, right_speed);
                                robot->turn_leds.set_led(0, correction < -10);
                                robot->turn_leds.set_led(1, correction > 10);
                            }

                            vTaskDelay(pdMS_TO_TICKS(30));
                        } },
                    "Test", 192, this, 1, nullptr);
    }
};

#endif
