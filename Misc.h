// Misc.h

#ifndef MISC_H
#define MISC_H

#include "Settings.h"
#include "TBTracker.h"

#include <Arduino.h>

void createRTTYTXLine(const char *PayloadID, unsigned long aCounter, const char *aPrefix);

void createLoRaTXLine(const char *PayloadID, unsigned long aCounter, const char *aPrefix);

char hexConvert(char Character);

void EEPROMWritelong(int address, long value);

long EEPROMReadlong(long address);

void resetTransmissionCounters();

extern char Sentence[SENTENCE_LENGTH];
extern TGPS UGPS;

#endif