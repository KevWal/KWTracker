// MFSK Modulation

#include "Arduino.h"
#include "Settings.h"
#include "Horus_L1.h"

uint32_t fsk4_base = 0, fsk4_baseHz = 0;
uint32_t fsk4_shift = 0, fsk4_shiftHz = 0;
//uint32_t fsk4_bitDuration = 0;
uint32_t fsk4_tones[4];
uint32_t fsk4_tonesHz[4];

volatile bool proceed = false; // ISR timing flag

// ISR timer based tx routines


// Prescaler of 8 works for my scenarios:
// Clock Mhz	  50	        100	        300 Baud
// 1,843,200    4,607.00    2,303.00    767.00 
// 14,745,600   36,863.00   18,431.00   6,143.00 
// 16,000,000   39,999.00   19,999.00   6,665.67
// Timer1 max = 65k


// Set up Timer1 for interrupts every symbol period
void isr_timer1_start() {
  noInterrupts();                                
  TCCR1A = 0;                                    // Set entire TCCR1A register to 0 = Disconnect interrupt output pins, sets normal waveform mode. We're just using Timer1 as a counter.
  TCNT1  = 0;                                    // Initialize counter value to 0.
  TCCR1B = (1 << CS11) |                         // Set CS11 bit to set prescale to /8
    (1 << WGM12);                                // turn on CTC mode.
  //OCR1A = 4607;                                  // Set up interrupt trigger count = (mhz / (prescaler * baud)) - 1
  OCR1A = (F_CPU / (8 * FSK4_BAUD)) - 1;          // Set up interrupt trigger count = (mhz / (prescaler * baud)) - 1
  TIMSK1 = (1 << OCIE1A);                        // Enable timer compare interrupt.
  interrupts();                                  // Re-enable interrupts.
}

// Disable Timer1
void isr_timer1_stop() {
  noInterrupts();
  TCCR1B = 0x00;
  interrupts();
}

// Timer interrupt vector. This toggles the variable we use to gate each column of output to ensure accurate timing. 
// Called whenever Timer1 hits the count set in interruptSetup().
ISR(TIMER1_COMPA_vect){
  proceed = true;
} 



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
  uint32_t start = micros();

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

  // Use timer 1 interupt to get best timing possible for each transmission
  isr_timer1_start();

  fsk4_preamble(phy, 8);
  for(size_t i = 0; i < len; i++) {
    n += fsk4_writebyte(phy, buff[i]);
  }
  fsk4_standby(phy);

  isr_timer1_stop();

  return(n);
}
