// Sleep.h

#ifndef SLEEP_H
#define SLEEP_H

#include <Arduino.h>

void setup_Sleep(void);

void my_Sleep();

void setup_PowerPins();

void enable_PowerPins();

void disable_PowerPins();

extern volatile bool watchdogActivated;
extern volatile int sleepIterations;

#endif