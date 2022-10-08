// Sleep.h

#ifndef SLEEP_H
#define SLEEP_H

#include <Arduino.h>

void setupSleep(void);

void mySleep();

void setupPowerPins();

void enablePowerPins();

void disablePowerPins();

extern volatile bool watchdogActivated;
extern volatile int sleepIterations;

#endif