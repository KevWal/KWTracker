// Horus_L1.h

#ifndef __HORUS_FSK4__
#define __HORUS_FSK4__

// My version of RadioLib using the proceed flag
// Need to #define RADIOLIB_INTERRUPT_TIMING in BuildOpt.h
#include <RadioLibKW.h>

#include <Arduino.h>

// MFSK Modulation

int16_t fsk4_setup(PhysicalLayer* phy, float base, uint32_t shift, uint16_t rate);

void fsk4_idle(PhysicalLayer* phy);

void fsk4_preamble(PhysicalLayer* phy, uint8_t len);

size_t fsk4_write(PhysicalLayer* phy, uint8_t* buff, size_t len);

#endif
