#ifndef COMPONENT_H
#define COMPONENT_H

#include <Arduino.h>

typedef int pin;

class Component
{
public:
    virtual void setup() = 0;
};

#endif // COMPONENT_H