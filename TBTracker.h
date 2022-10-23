// TBTracker.h

#ifndef TBTRACKER_H
#define TBTRACKER_H

#include "Settings.h"
#include <Arduino.h>

/***********************************************************************************
* DATA STRUCTS
*  
* Normally no change necessary
************************************************************************************/
// Struct to hold GPS data
struct TGPS
{
  int Hours, Minutes, Seconds;
  float Longitude, Latitude;
  long Altitude;
  unsigned int Satellites;
  uint8_t FlightMode;
  double Speed;
};

#endif