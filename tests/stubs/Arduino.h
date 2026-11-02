#ifndef TEST_ARDUINO_H
#define TEST_ARDUINO_H
#include <stddef.h>
#include <stdlib.h>
#define INPUT 0
#define OUTPUT 1
#define LOW 0
#define HIGH 1
#define constrain(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
int analogRead(int pin);
void pinMode(int pin, int mode);
void analogWrite(int pin, int value);
void digitalWrite(int pin, int value);
unsigned long millis();
#endif
