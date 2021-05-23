// GPS.h

#ifndef GPS_H
#define GPS_H

#include "TBTracker.h"
#include "Settings.h"
#include <Arduino.h>

#define PEDESTRIAN 3
#define AIRBORNE 6 

void CheckGPS();

void smartDelay(unsigned long ms);

void processGPSData();

void printGPSData();

void SendUBX(unsigned char *Message, int Length);

void setDesiredMode(uint8_t aDesiredMode);

extern TGPS UGPS;

//If SerialGPS is defined then we are using a Hardware serial port and dont need Software Serial
#ifndef SERIALGPS
  #include <SoftwareSerial.h>
  extern SoftwareSerial SERIALGPS;
#endif

#endif