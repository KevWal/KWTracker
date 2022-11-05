// Horus_L1.cpp

/***********************************************************************************
* #includes
*
* #include user defined headers, followed by 3rd party library headers, then standard 
* library headers, with the headers in each section sorted alphabetically.
************************************************************************************/
#include "Horus_L1.h"

#include "Settings.h"

#include <Arduino.h>


uint32_t fsk4_base = 0, fsk4_baseHz = 0;
uint32_t fsk4_shift = 0, fsk4_shiftHz = 0;
//uint32_t fsk4_bitDuration = 0;
uint32_t fsk4_tones[4];
uint32_t fsk4_tonesHz[4];

volatile bool proceed = false; // ISR timing flag


int16_t fsk4_setup(PhysicalLayer* phy, float base, uint32_t shift, uint16_t rate){
 // save configuration
  fsk4_baseHz = base;
  fsk4_shiftHz = shift;

  // calculate duration of 1 bit
  //KW replaced by Interupt drive code
  //fsk4_bitDuration = (uint32_t)1000000/rate;

  // calculate module carrier frequency resolution
  uint32_t step = round(phy->getFreqStep());

  // check minimum shift value
  if(shift < step / 2) {
    return 0;
  }

  // round shift to multiples of frequency step size
  if(shift % step < (step / 2)) {
    fsk4_shift = shift / step;
  } else {
    fsk4_shift = (shift / step) + 1;
  }

  // Write resultant tones into arrays for quick lookup when modulating.
  fsk4_tones[0] = 0;
  fsk4_tones[1] = fsk4_shift;
  fsk4_tones[2] = fsk4_shift*2;
  fsk4_tones[3] = fsk4_shift*3;

  // calculate 24-bit frequency
  fsk4_base = (base * 1000000.0) / phy->getFreqStep();

  //Serial.println(fsk4_base);

  // configure for direct mode
  return(phy->startDirect());

}

int16_t fsk4_transmitDirect(PhysicalLayer* phy, uint32_t freq) {
  return(phy->transmitDirect(freq));
}

void fsk4_tone(PhysicalLayer* phy, uint8_t i) {
  //uint32_t start = micros();

  proceed = false; while (!proceed) { yield(); } // Wait for the right time to start the next tone

#ifdef DEVTIMING
  PINB = 0b00010000; //digitalWrite(DEVTIMING, !digitalRead(DEVTIMING));
#endif
  fsk4_transmitDirect(phy, fsk4_base + fsk4_tones[i]);

  //delayMicroseconds(wait);
  //while(micros() - start < fsk4_bitDuration) {
  //  yield();
  //}

}

void fsk4_idle(PhysicalLayer* phy){
    //fsk4_tone(phy, 0);
    fsk4_transmitDirect(phy, fsk4_base + fsk4_tones[0]);
}

void fsk4_standby(PhysicalLayer* phy){
    proceed = false; while (!proceed) {yield();} // Make sure the last tone lasts the required amount of time.
#ifdef DEVTIMING
  PINB = 0b00010000; //digitalWrite(DEVTIMING, !digitalRead(DEVTIMING));
#endif
    phy->standby();
}

size_t fsk4_writebyte(PhysicalLayer* phy, uint8_t b){
    int k;
    // Send symbols MSB first.
    for (k=0;k<4;k++)
    {
        // Extract 4FSK symbol (2 bits)
        uint8_t symbol = (b & 0xC0) >> 6;
        // Modulate
        fsk4_tone(phy, symbol);
        // Shift to next symbol.
        b = b << 2;
    }

  return(1);
}

void fsk4_preamble(PhysicalLayer* phy, uint8_t len){
    int k;
    for (k=0; k<len; k++){
        fsk4_writebyte(phy, 0x1B);
    }
}

size_t fsk4_write(PhysicalLayer* phy, uint8_t* buff, size_t len){
  size_t n = 0;

  fsk4_preamble(phy, 8);
  for(size_t i = 0; i < len; i++) {
    n += fsk4_writebyte(phy, buff[i]);
  }
  fsk4_standby(phy);
  return(n);
}
