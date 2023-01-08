// Misc.cpp

/***********************************************************************************
* #includes
*
* #include user defined headers, followed by 3rd party library headers, then standard 
* library headers, with the headers in each section sorted alphabetically.
************************************************************************************/
#include "Misc.h"

#include "ADC.h"
#include "KWTracker.h"
#include "Radio.h"

#include <Print.h>
#include <util/crc16.h>
#include <EEPROM.h>

extern TGPS UGPS;
extern char Sentence[SENTENCE_LENGTH];

/***********************************************************************************
* CreateTXLine generates two payload sentences to transmit. One
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
void createLoRaTXLine(const char *PayloadID, unsigned long aCounter, const char *aPrefix)
{
   int Count, i, j;
   unsigned int CRC;
   char LatitudeString[16], LongitudeString[16], HeadingString[16];
   char InternalTemp[10];
   char VccVoltage[10];
   char BattVoltage[10];

#ifdef ATMEGA1284P // ATMEGA1284P does not have an internal temperature sensor
  dtostrf(getRadioTemp(), 3, 1, InternalTemp);
#else // Get the internal chip temperature
  dtostrf(readTemp(), 3, 1, InternalTemp);
#endif
  DBGPRNTST(F("InternalTemp: ")); DBGPRNTLN(InternalTemp);

  // Get the battery voltage
  dtostrf(readVCC(), 4, 2, VccVoltage);
  DBGPRNTST(F("VccVoltage: ")); DBGPRNTLN(VccVoltage);

  // Get the external voltage
  dtostrf(readExternalVoltage(), 4, 2, BattVoltage);
  DBGPRNTST(F("BattVoltage: ")); DBGPRNTLN(BattVoltage);

  dtostrf(UGPS.Heading, 0, 0, HeadingString);
  DBGPRNTST(F("Heading: ")); DBGPRNTLN(HeadingString);

  dtostrf(UGPS.Latitude, 7, 5, LatitudeString);
  dtostrf(UGPS.Longitude, 7, 5, LongitudeString);   
   
  sprintf(Sentence,
#if defined(USE_FIELDSTR)
            "%s%s,%ld,%02d:%02d:%02d,%s,%s,%ld,%u,%s,%s,%s,%s,%s",
#else
            "%s%s,%ld,%02d:%02d:%02d,%s,%s,%ld,%u,%s,%s,%s,%s",
#endif
            aPrefix,
            PayloadID,
            aCounter,
            UGPS.Hours, UGPS.Minutes, UGPS.Seconds,   
            LatitudeString,
            LongitudeString,
            UGPS.Altitude,
            UGPS.Satellites,
            HeadingString,
            InternalTemp,
            BattVoltage,
            VccVoltage
#if defined(USE_FIELDSTR)            
            ,
            FIELDSTR
#endif
  );

  Count = strlen(Sentence);

  // Calc UKHAS CRC
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

  Sentence[Count++] = '*'; // Count++ accesses Sentence[Count] and then increments Count
  Sentence[Count++] = hexConvert((CRC >> 12) & 15);
  Sentence[Count++] = hexConvert((CRC >> 8) & 15);
  Sentence[Count++] = hexConvert((CRC >> 4) & 15);
  Sentence[Count++] = hexConvert(CRC & 15);
  Sentence[Count++] = '\n';
  Sentence[Count++] = '\0';

  DBGPRNTST(F("TX Line: ")); DBGPRNT(Sentence);
}


void createRTTYTXLine(const char *PayloadID, unsigned long aCounter, const char *aPrefix)
{
   int Count, i, j;
   unsigned int CRC;
   char LatitudeString[16], LongitudeString[16];

  dtostrf(UGPS.Latitude, 7, 5, LatitudeString);
  dtostrf(UGPS.Longitude, 7, 5, LongitudeString);   
   
  sprintf(Sentence,
            "%s%s,%ld,%02d:%02d:%02d,%s,%s,%ld,%u",
            aPrefix,
            PayloadID,
            aCounter,
            UGPS.Hours, UGPS.Minutes, UGPS.Seconds,   
            LatitudeString,
            LongitudeString,
            UGPS.Altitude,
            UGPS.Satellites
  );

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
  Sentence[Count++] = hexConvert((CRC >> 12) & 15);
  Sentence[Count++] = hexConvert((CRC >> 8) & 15);
  Sentence[Count++] = hexConvert((CRC >> 4) & 15);
  Sentence[Count++] = hexConvert(CRC & 15);
  Sentence[Count++] = '\n';  
  Sentence[Count++] = '\0';

  DBGPRNTST(F("TX Line: ")); DBGPRNT(Sentence);
}




int createFSK4TXLine(uint16_t PayloadID, unsigned long aCounter){
  // Generate a Horus Binary v2 packet, and populate it with data.

  struct HorusBinaryPacketV2 BinaryPacketV2;

  BinaryPacketV2.PayloadID = PayloadID; // 255 = 4FSKTEST-V2. Refer https://github.com/projecthorus/horusdemodlib/blob/master/payload_id_list.txt
  BinaryPacketV2.Counter = aCounter;
  BinaryPacketV2.Hours = UGPS.Hours;
  BinaryPacketV2.Minutes = UGPS.Minutes;
  BinaryPacketV2.Seconds = UGPS.Seconds;
  BinaryPacketV2.Latitude = UGPS.Latitude;
  BinaryPacketV2.Longitude = UGPS.Longitude;
  BinaryPacketV2.Altitude = UGPS.Altitude;
  BinaryPacketV2.Speed = UGPS.Speed;

  DBGPRNTST(F("BattVoltageRaw: ")); DBGPRNTLN(readExternalVoltage());
  // BattVoltage 0 = 0.5v, 255 = 2.0V, linear steps in-between
  BinaryPacketV2.BattVoltage = (uint8_t) (readExternalVoltage() * 51); // 255 = 5.0v
  DBGPRNTST(F("EncodedBattVoltage: ")); DBGPRNTLN(BinaryPacketV2.BattVoltage);

  BinaryPacketV2.Sats = UGPS.Satellites;

#ifdef ATMEGA1284P // ATMEGA1284P does not have an internal temperature sensor
  BinaryPacketV2.Temp = getRadioTemp();
#else // Get the internal chip temperature
  BinaryPacketV2.Temp = readTemp();
#endif

  // Custom section. This is an example only, and the 9 bytes in this section can be used in other
  // ways. Refer here for details: https://github.com/projecthorus/horusdemodlib/wiki/5-Customising-a-Horus-Binary-v2-Packet
  BinaryPacketV2.dummy1 = 0;        // uint8
  BinaryPacketV2.dummy2 = 0.0;  // float32
  BinaryPacketV2.dummy3 = 0;      // uint8 - interpreted as a battery voltage 0-5V
  BinaryPacketV2.dummy4 = 0;      // uint8 - interpreted as a fixed-point value (div/10)
  BinaryPacketV2.dummy5 = 0;     // uint16 - interpreted as a fixed-point value (div/100)

  BinaryPacketV2.Checksum = (uint16_t) crc16((unsigned char*) &BinaryPacketV2, sizeof(BinaryPacketV2) - sizeof(BinaryPacketV2.Checksum));
  DBGPRNTST(F("Checksum: ")); DBGPRNTLN(BinaryPacketV2.Checksum, HEX);

  memcpy(Sentence, &BinaryPacketV2, sizeof(BinaryPacketV2));

  return sizeof(struct HorusBinaryPacketV2);
}


//===============================================================================
// Fast CRC16 code, using Atmel's optimized libraries!
unsigned int crc16(unsigned char *string, unsigned int len) {
	unsigned int i;
	unsigned int crc;
	crc = 0xFFFF; // Standard CCITT seed for CRC16.
	// Calculate the sum, ignore $ sign's
	for (i = 0; i < len; i++) {
		crc = _crc_xmodem_update(crc,(uint8_t)string[i]);
	}
	return crc;
}


//===============================================================================
char hexConvert(char Character)
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
   EEPROMWritelong(0x08,0);
}