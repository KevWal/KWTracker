// GPS.cpp

/***********************************************************************************
* #includes
*
* #include user defined headers, followed by 3rd party library headers, then standard 
* library headers, with the headers in each section sorted alphabetically.
************************************************************************************/
#include "GPS.h"
#include "TBTracker.h"
#include "Settings.h"

#include <TinyGPS++.h>


// The TinyGPS++ object
TinyGPSPlus gps;

/*********************************************************************************************************************************/
void checkGPS()
{    
  processGPSData();
  printGPSData();
}


/*********************************************************************************************************************************/
// This custom version of delay() ensures that the gps object is being "fed".
void smartDelay(unsigned long ms)
{
  DBGPRNTST(F("Reading GPS ... "));
  unsigned long start = millis();

  do 
  {
    // Read from GPS and feed into TinyGPS++
    if (SERIALGPS.available()) gps.encode(SERIALGPS.read());
    // Toggle LED fast if less than 4 sats, slow if more than 4 sats.
    if (gps.satellites.value() < 5 && gps.altitude.meters() < 1000) digitalWrite(LED, millis()%150>75);
    if (gps.satellites.value() >= 5 && gps.altitude.meters() < 1000) digitalWrite(LED, millis()%1000>500);
  } while (millis() - start < ms);
  digitalWrite(LED, LOW);
  DBGPRNT(gps.charsProcessed()); DBGPRNT(F(" chars read, ")); DBGPRNT(gps.passedChecksum()); DBGPRNTLN(F(" valid sentances."));
}

/*********************************************************************************************************************************/
void processGPSData()
{
  uint8_t DesiredMode;

  DBGPRNTST(F("Processing GPS ... "));
  
  // Number of Satellites
  if (gps.satellites.isValid())
    UGPS.Satellites = gps.satellites.value();
  else
  {
    UGPS.Satellites = 0;
    DBGPRNT(F("Sats not valid, "));
  }

 // Time
 if (gps.time.isValid())
 {
    UGPS.Hours = gps.time.hour();
    UGPS.Minutes = gps.time.minute();
    UGPS.Seconds = gps.time.second();
 }
 else
 {
    UGPS.Hours = 0;
    UGPS.Minutes = 0;
    UGPS.Seconds = 0;
    DBGPRNT(F("Time not valid, "));
 }

 // Position
 if (gps.location.isValid())
 {
    UGPS.Longitude = gps.location.lng();
    UGPS.Latitude = gps.location.lat();
 }
 else
 {
   UGPS.Longitude = 0;
   UGPS.Latitude = 0;
   DBGPRNT(F("Location not valid, "));
 }

 // Altitude
 if (gps.altitude.isValid())
    UGPS.Altitude = gps.altitude.meters();
 else
 {
   UGPS.Altitude = 0;
   DBGPRNT(F("Alt not valid, "));
 }

// Speed
if (gps.speed.isValid())
    UGPS.Speed = gps.speed.kmph();
 else
 {
   UGPS.Speed = 0;
   DBGPRNT(F("Speed not valid, "));
 }

 if (UGPS.Altitude < 0)
   UGPS.Altitude = 0;

 if (UGPS.Altitude > 1000)
   DesiredMode = AIRBORNE;
 else 
   DesiredMode = PEDESTRIAN;

  DBGPRNTLN(F("done."));

 // Set the correct flight mode for high altitude
 if (UGPS.FlightMode != DesiredMode)
   setDesiredMode(DesiredMode);   
  
}

/*********************************************************************************************************************************/
void printGPSData()
{
  DBGPRNTST(UGPS.Hours); DBGPRNT(":"); DBGPRNT(UGPS.Minutes); DBGPRNT(":"); DBGPRNT(UGPS.Seconds);
  DBGPRNT(" "); DBGPRNT(UGPS.Latitude, 6); DBGPRNT(" "); DBGPRNT(UGPS.Longitude, 6);
  DBGPRNT(" "); DBGPRNT(UGPS.Altitude); DBGPRNT(" "); DBGPRNTLN(UGPS.Satellites);
}


/*********************************************************************************************************************************/
void sendUBX(unsigned char *Message, int Length)
{   
  for (int i=0; i<Length; i++)
  {
    SERIALGPS.write(Message[i]);
  }
}


/*********************************************************************************************************************************/
// Pedestrian mode
void setGPS_DynamicModel3()
{
  unsigned char setNav[] = 
  {
     0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 
     0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4A, 0x75
   };  

  DBGPRNTST(F("Setting pedestrian mode ... "));
  sendUBX(setNav, sizeof(setNav));
  DBGPRNTLN(F("Done"));
}

/*********************************************************************************************************************************/
// High altitude flightmode
void setGPS_DynamicModel6()
{
  unsigned char setNav[] = 
  {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 
    0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4D, 0xDB
  };

  DBGPRNTST(F("Setting airborne mode ... "));
  sendUBX(setNav, sizeof(setNav));
  DBGPRNTLN(F("Done"));
}

/*********************************************************************************************************************************/
void setDesiredMode(uint8_t aDesiredMode)
{
  if (aDesiredMode == PEDESTRIAN)
    setGPS_DynamicModel3();
  else if (aDesiredMode == AIRBORNE)
    setGPS_DynamicModel6();
 
  UGPS.FlightMode = aDesiredMode;
}