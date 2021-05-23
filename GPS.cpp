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
void CheckGPS()
{    
  processGPSData();
  printGPSData();
}


/*********************************************************************************************************************************/
// This custom version of delay() ensures that the gps object is being "fed".
void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (SERIALGPS.available())
      gps.encode(SERIALGPS.read());
  } while (millis() - start < ms);
}

/*********************************************************************************************************************************/
void processGPSData()
{
  uint8_t DesiredMode;
  
  // Number of Satellites
  if (gps.satellites.isValid())
    UGPS.Satellites = gps.satellites.value();
  else
    UGPS.Satellites = 0;

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
 }

 // Altitude
 if (gps.altitude.isValid())
    UGPS.Altitude = gps.altitude.meters();
 else
    UGPS.Altitude = 0;    

 if (UGPS.Altitude < 0)
   UGPS.Altitude = 0;    

 if (UGPS.Altitude > 1000)
   DesiredMode = AIRBORNE;
 else 
   DesiredMode = PEDESTRIAN;

 // Set the correct flight mode for high altitude
 if (UGPS.FlightMode != DesiredMode)
   setDesiredMode(DesiredMode);
    
}

/*********************************************************************************************************************************/
void printGPSData()
{
#if defined(DEVMODE)
  SERIALDBG.print("         Time: "); SERIALDBG.print(UGPS.Hours); SERIALDBG.print(":"); SERIALDBG.print(UGPS.Minutes); SERIALDBG.print(":"); SERIALDBG.println(UGPS.Seconds);
  SERIALDBG.print("     Latitude: "); SERIALDBG.println(UGPS.Latitude, 6);
  SERIALDBG.print("    Longitude: "); SERIALDBG.println(UGPS.Longitude, 6);
  SERIALDBG.print(" Altitude (m): "); SERIALDBG.println(UGPS.Altitude);
  SERIALDBG.print("   Satellites: "); SERIALDBG.println(UGPS.Satellites);
  SERIALDBG.println();
  SERIALDBG.println("-------------------------");
#endif
}


/*********************************************************************************************************************************/
void SendUBX(unsigned char *Message, int Length)
{  
  int i;
  
  for (i=0; i<Length; i++)
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

#if defined(DEVMODE)
  SERIALDBG.print("Setting pedestrian mode...");
#endif
  SendUBX(setNav, sizeof(setNav));
#if defined(DEVMODE)
  SERIALDBG.println("Done");
#endif
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

#if defined(DEVMODE)  
  SERIALDBG.print("Setting airborne mode...");
#endif
  SendUBX(setNav, sizeof(setNav));
#if defined(DEVMODE)  
  SERIALDBG.println("Done");
#endif
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