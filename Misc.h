// Misc.h

#ifndef MISC_H
#define MISC_H

#include "Settings.h"
#include "TBTracker.h"

#include <Arduino.h>

void createRTTYTXLine(const char *PayloadID, unsigned long aCounter, const char *aPrefix);

void createLoRaTXLine(const char *PayloadID, unsigned long aCounter, const char *aPrefix);

int createFSK4TXLine(uint16_t PayloadID, unsigned long aCounter);

unsigned int crc16(unsigned char *string, unsigned int len);

char hexConvert(char Character);

void EEPROMWritelong(int address, long value);

long EEPROMReadlong(long address);

void resetTransmissionCounters();

extern char Sentence[SENTENCE_LENGTH];
extern TGPS UGPS;

// Horus v2 Mode 1 (32-byte) Binary Packet Struct
struct HorusBinaryPacketV2
{
    uint16_t  PayloadID;
    uint16_t  Counter;
    uint8_t   Hours;
    uint8_t   Minutes;
    uint8_t   Seconds;
    float     Latitude;
    float     Longitude;
    uint16_t  Altitude;
    uint8_t   Speed;       // Speed in Knots (1-255 knots)
    uint8_t   Sats;
    int8_t    Temp;        // Twos Complement Temp value.
    uint8_t   BattVoltage; // 0 = 0.5v, 255 = 2.0V, linear steps in-between.
    // The following 9 bytes (up to the CRC) are user-customizable. The following just
    // provides an example of how they could be used.
    uint8_t     dummy1;      // unsigned int
    float     dummy2;       // Float 
    uint8_t     dummy3;     // battery voltage test
    uint8_t     dummy4;     // divide by 10
    uint16_t     dummy5;    // divide by 100
    uint16_t    Checksum;    // CRC16-CCITT Checksum.
}  __attribute__ ((packed));

#endif