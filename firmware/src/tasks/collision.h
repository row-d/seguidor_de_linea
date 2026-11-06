#ifndef COLLISION_TASK_H
#define COLLISION_TASK_H

#include "robot.h"

// Agarra el objeto, lo deja a la derecha de la linea y vuelve a orientarse hacia ella.
template <size_t N>
void Robot<N>::remove_object()
{
    const int approach_speed = 110;
    const long grab_distance_cm = 5;
    const unsigned long approach_timeout_ms = 1500;
    const uint32_t claw_move_ms = 500;
    const uint32_t object_turn_ms = 500;

    movement.stop();
    logger.event("object_detected");
    claw.open();
    wait_ms(claw_move_ms);

    const unsigned long approach_start = millis();
    movement.drive(approach_speed, approach_speed);
    while (obstacle_sensor.read_distance() > grab_distance_cm &&
           millis() - approach_start < approach_timeout_ms)
    {
        vTaskDelay(1);
    }
    movement.stop();

    claw.close();
    wait_ms(claw_move_ms);

    movement.rotate_right(TURN_SPEED);
    wait_ms(object_turn_ms);
    movement.stop();

    claw.open();
    wait_ms(claw_move_ms);
    buzzer.beep(200);
    logger.event("object_removed");

    movement.rotate_left(TURN_SPEED);
    wait_ms(object_turn_ms);
    movement.stop();
}

#endif
