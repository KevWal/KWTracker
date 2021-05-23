// GPS.h

#ifndef GPS_H
#define GPS_H

#include "TBTracker.h"

#include <SoftwareSerial.h>
#include <Arduino.h>

#define PEDESTRIAN 3
#define AIRBORNE 6 

void CheckGPS();

void smartDelay(unsigned long ms);

void processGPSData();

void printGPSData();

void SendUBX(unsigned char *Message, int Length);

void setDesiredMode(uint8_t aDesiredMode);

extern SoftwareSerial SerialGPS;
extern TGPS UGPS;

#endif