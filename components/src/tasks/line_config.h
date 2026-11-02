#ifndef LINE_CONFIG_H
#define LINE_CONFIG_H

struct LineControlConfig
{
    int base_speed = 90;
    int max_correction = 90;
    int search_speed = 70;
    double kp = 0.06;
    double ki = 0.0;
    double kd = 0.0; // Tune P first, then add D on the real robot.
    unsigned int period_ms = 30;
    unsigned int sensor_timeout_ms = 150;
    unsigned int search_switch_ms = 300;
    unsigned int search_timeout_ms = 2000;
    bool calibration_mode = false;
    bool diagnostics = true;
};

#endif
