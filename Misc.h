// Misc.h

#ifndef MISC_H
#define MISC_H

#include "Settings.h"
#include "TBTracker.h"

#include <Arduino.h>

void CreateTXLine(const char *PayloadID, unsigned long aCounter, const char *aPrefix);

char Hex(char Character);

void EEPROMWritelong(int address, long value);

long EEPROMReadlong(long address);

void resetTransmissionCounters();

extern char Sentence[SENTENCE_LENGTH];
extern TGPS UGPS;

#endif