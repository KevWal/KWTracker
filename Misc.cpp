// Misc.cpp

/***********************************************************************************
* #includes
*
* #include user defined headers, followed by 3rd party library headers, then standard 
* library headers, with the headers in each section sorted alphabetically.
************************************************************************************/
#include "Misc.h"
#include "ADC.h"
#include "TBTracker.h"

#include <EEPROM.h>

/***********************************************************************************
* The function CreateTXLine generates two payload sentences to transmit. One
* for RTTY and one for LoRa. These sentences are compatible with the habhub tracker.
* 
* Run the software and decode your payload sentences. 
*
* Sample of a RTTY payload sentence this software will generate:
* 
* $$RTTY-ID,5,12:11:10,53.16307,6.16444,12,3,12.0,4.61,3.71*109F 
* 
* This sentence contains the following fields, separated by comma's:
*     $$        = prefix
*     RTTY-ID   = callsign
*     5         = sequence number
*     12:11:10  = Time (UTC)
*     53.163067 = Latitude
*     6.16444   = Longitude
*     12        = Altitude
*     3         = Number of satellites
*     12.0      = Internal (chip) temperature in Celsius
*     4.61      = VCC voltage
*     3.71      = External voltage on analog pin A1
*     *         = seperator
*     109F      = Checksum
*     
* The LoRa payload looks the same, except for the callsign (if you changed that).     
************************************************************************************/
//===============================================================================
void CreateTXLine(const char *PayloadID, unsigned long aCounter, const char *aPrefix)
{
   int Count, i, j;
   unsigned int CRC;
   char LatitudeString[16], LongitudeString[16];
   char InternalTemp[10];
   char BattVoltage[10];
   char ExtVoltage[10];

#ifdef ATMEGA1284P // ATMEGA1284P does not have an internal temperature sensor
  dtostrf(0.0, 3, 1, InternalTemp);
#else // Get the internal chip temperature
  dtostrf(ReadTemp(), 3, 1, InternalTemp);
  DBGPRNTST(F("InternalTemp: ")); DBGPRNTLN(InternalTemp);
#endif

  // Get the battery voltage
  dtostrf(ReadVCC(), 4, 2, BattVoltage);
  DBGPRNTST(F("BattVoltage: ")); DBGPRNTLN(BattVoltage);

  // Get the external voltage
  dtostrf(ReadExternalVoltage(), 4, 2, ExtVoltage);
  DBGPRNTST(F("ExtVoltage: ")); DBGPRNTLN(ExtVoltage);
         
  dtostrf(UGPS.Latitude, 7, 5, LatitudeString);
  dtostrf(UGPS.Longitude, 7, 5, LongitudeString);   
   
  sprintf(Sentence,
            "%s%s,%ld,%02d:%02d:%02d,%s,%s,%ld,%u,%s,%s,%s",
            aPrefix,
            PayloadID,
            aCounter,
            UGPS.Hours, UGPS.Minutes, UGPS.Seconds,   
            LatitudeString,
            LongitudeString,
            UGPS.Altitude,
            UGPS.Satellites,
            InternalTemp,
            BattVoltage,
            ExtVoltage);

  Count = strlen(Sentence);

  // Calc CRC
  CRC = 0xffff;           // Seed
   
  for (i = strlen(aPrefix); i < Count; i++)
  {   // For speed, repeat calculation instead of looping for each bit
    CRC ^= (((unsigned int)Sentence[i]) << 8);
    for (j=0; j<8; j++)
    {
      if (CRC & 0x8000)
        CRC = (CRC << 1) ^ 0x1021;
      else
        CRC <<= 1;
    }
  }

  Sentence[Count++] = '*';
  Sentence[Count++] = Hex((CRC >> 12) & 15);
  Sentence[Count++] = Hex((CRC >> 8) & 15);
  Sentence[Count++] = Hex((CRC >> 4) & 15);
  Sentence[Count++] = Hex(CRC & 15);
  Sentence[Count++] = '\n';  
  Sentence[Count++] = '\0';

  DBGPRNTST(F("TX Line: ")); DBGPRNT(Sentence);
}

//===============================================================================
char Hex(char Character)
{
  char HexTable[] = "0123456789ABCDEF";
  
  return HexTable[(unsigned char)Character];
}


//===============================================================================
//This function will write a 4 byte (32bit) long to the eeprom at
//the specified address to address + 3.
void EEPROMWritelong(int address, long value)
      {
      //Decomposition from a long to 4 bytes by using bitshift.
      //One = Most significant -> Four = Least significant byte
      uint8_t four = (value & 0xFF);
      uint8_t three = ((value >> 8) & 0xFF);
      uint8_t two = ((value >> 16) & 0xFF);
      uint8_t one = ((value >> 24) & 0xFF);

      //Write the 4 bytes into the eeprom memory.
      EEPROM.write(address, four);
      EEPROM.write(address + 1, three);
      EEPROM.write(address + 2, two);
      EEPROM.write(address + 3, one);
      }

//===============================================================================
//This function will take and assembly 4 byte of the Eeprom memory in order to form a long variable.
long EEPROMReadlong(long address)
      {
      //Read the 4 bytes from the eeprom memory.
      long four = EEPROM.read(address);
      long three = EEPROM.read(address + 1);
      long two = EEPROM.read(address + 2);
      long one = EEPROM.read(address + 3);

      //Return the recomposed long by using bitshift.
      return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
      }

//===============================================================================
//This function will reset the transmission counters to 0.
void resetTransmissionCounters()
{
   EEPROMWritelong(0x00,0);
   EEPROMWritelong(0x04,0);
}
