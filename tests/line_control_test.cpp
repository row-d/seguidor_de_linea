#include <cassert>
#include <climits>
#include <iostream>
#include "components/line_sensors.h"
#include "components/movement.h"
#include "tasks/line_recovery.h"

static int adc[3] = {};
static int reads[3] = {};
static int modes[16] = {};
static int pwm[16] = {};
static int digital[16] = {};
int analogRead(int pin) { ++reads[pin]; return adc[pin]; }
void pinMode(int pin, int mode) { modes[pin] = mode; }
void analogWrite(int pin, int value) { pwm[pin] = value; }
void digitalWrite(int pin, int value) { digital[pin] = value; }
unsigned long millis() { return 123; }

static void values(int left, int center, int right)
{
    adc[0] = left; adc[1] = center; adc[2] = right;
    reads[0] = reads[1] = reads[2] = 0;
}

int main()
{
    const int pins[] = {0, 1, 2};
    const int thresholds[] = {110, 120, 100};
    LineSensors<3> sensors(pins, thresholds);
    sensors.setup();
    assert(modes[0] == INPUT && modes[1] == INPUT && modes[2] == INPUT);

    values(65, 700, 50);
    auto sample = sensors.sample();
    assert(sample.valid && sample.found && !sample.ambiguous && sample.error == 0);
    assert(sample.captured_ms == 123);
    assert(reads[0] == 1 && reads[1] == 1 && reads[2] == 1);
    values(700, 58, 50); assert(sensors.sample().error == -1000);
    values(65, 58, 700); assert(sensors.sample().error == 1000);
    values(65, 58, 50); assert(!sensors.sample().found);
    values(700, 58, 700); assert(sensors.sample().ambiguous);
    values(700, 700, 700); assert(sensors.sample().ambiguous);
    values(-1, 700, 50); assert(!sensors.sample().valid);

    const int background[] = {60, 55, 50};
    const int black[] = {850, 700, 650};
    assert(sensors.set_calibration(background, black));
    values(850, 55, 650);
    sample = sensors.sample();
    assert(sample.strength[0] == 1000 && sample.strength[2] == 1000 && sample.error == 0);
    values(850, 700, 50);
    sample = sensors.sample();
    assert(sample.error == -500 && !sample.ambiguous);
    const int poor_black[] = {70, 700, 650};
    assert(!sensors.set_calibration(background, poor_black));
    values(60, 700, 50); assert(sensors.sample().error == 0);

    const int inverted_background[] = {850, 700, 650};
    const int inverted_black[] = {60, 55, 50};
    assert(sensors.set_calibration(inverted_background, inverted_black));
    values(850, 55, 650);
    assert(sensors.sample().found && sensors.sample().error == 0);
    values(850, 700, 650); assert(!sensors.sample().found);
    LineSensors<3> low_on_black(pins, thresholds, false);
    values(500, 40, 500); assert(low_on_black.sample().found);

    LineRecovery recovery;
    assert(recovery.direction(0, 300, 2000) == -1);
    assert(recovery.direction(299, 300, 2000) == -1);
    assert(recovery.direction(300, 300, 2000) == 1);
    assert(recovery.direction(600, 300, 2000) == -1);
    assert(recovery.direction(2000, 300, 2000) == 0);
    assert(recovery.direction(2300, 300, 2000) == 0);
    recovery.observe(500);
    assert(recovery.direction(2400, 300, 2000) == 1);
    assert(recovery.direction(2700, 300, 2000) == -1);
    recovery.reset();
    assert(recovery.direction(ULONG_MAX - 100, 300, 2000) == 1);
    assert(recovery.direction(199, 300, 2000) == -1);

    Movement movement(2, 3, 4, 5);
    movement.setup();
    movement.move_forward(500, -10);
    assert(pwm[2] == 255 && pwm[4] == 0);
    movement.move_left(90);
    assert(pwm[2] == 0 && pwm[4] == 90);
    movement.move_right(90);
    assert(pwm[2] == 90 && pwm[4] == 0);
    movement.rotate_left(70);
    assert(digital[3] == HIGH && digital[5] == HIGH);
    movement.rotate_right(70);
    assert(digital[3] == LOW && digital[5] == LOW);
    movement.stop();
    assert(pwm[2] == 0 && pwm[4] == 0);
    std::cout << "OK: sensors, calibration, recovery and motor commands\n";
}
