// Radio.cpp

/***********************************************************************************
* #includes
*
* #include user defined headers, followed by 3rd party library headers, then standard 
* library headers, with the headers in each section sorted alphabetically.
************************************************************************************/
#include "Radio.h"

#include "Settings.h"

// Need to #define RADIOLIB_INTERRUPT_TIMING in BuildOpt.h
#include <RadioLib.h>

#include <Arduino.h>

// Change 'SX1278' in the line below to 'SX1276' if you have a SX1276 module.
SX1278 radio = new Module(PIN_NSS, PIN_DIO0, PIN_RESET, PIN_DIO1);

// create RTTY client instance using the radio module
RTTYClient rtty(&radio);

// create FSK4 client instance using the FSK module
FSK4Client fsk4(&radio);

//===============================================================================
// ISR timer based tx routines

// Prescaler of 8 works for my scenarios:
// Clock Mhz	  50	        100	        300 Baud
// 1,843,200    4,607.00    2,303.00    767.00 
// 14,745,600   36,863.00   18,431.00   6,143.00 
// 16,000,000   39,999.00   19,999.00   6,665.67
// Timer1 max = 65k

// Set up Timer1 for interrupts every symbol period
void isr_timer1_start(uint16_t baud) {
  noInterrupts();                                
  TCCR1A = 0;                                    // Set entire TCCR1A register to 0 = Disconnect interrupt output pins, sets normal waveform mode. We're just using Timer1 as a counter.
  TCNT1  = 0;                                    // Initialize counter value to 0.
  TCCR1B = (1 << CS11) |                         // Set CS11 bit to set prescale to /8
    (1 << WGM12);                                // turn on CTC mode.
  //OCR1A = 4607;                                // Set up interrupt trigger count = (mhz / (prescaler * baud)) - 1
  OCR1A = (F_CPU / (8 * baud)) - 1;              // Set up interrupt trigger count = (mhz / (prescaler * baud)) - 1
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
  radio.setTimerFlag();
} 


//===============================================================================
void setupFSK()
{
  // Reset the radio
  resetRadio();

  // Initialize FSK
  DBGPRNTST(F("[FSK] Initializing ... "));

  int16_t state = radio.beginFSK(FSK_FREQUENCY, FSK_BITRATE, FSK_FREQDEV, FSK_RXBANDWIDTH, FSK_POWER, FSK_PREAMBLELENGTH, FSK_ENABLEOOK);
  if (state == RADIOLIB_ERR_NONE) DBGPRNTLN(F(" success!")); else { DBGPRNT(F(" failed, code: ")); DBGPRNTLN(state); }

  // set over current protection limit to 80 mA (accepted range is 45 - 240 mA)
  // NOTE: set value to 0 to disable overcurrent protection
  if (radio.setCurrentLimit(FSK_CURRENTLIMIT) == RADIOLIB_ERR_INVALID_CURRENT_LIMIT) {
    DBGPRNTSTLN(F("Selected current limit is invalid for this module!"));
  }
  
}


//===============================================================================
void setupRTTY()
{
  // First setup FSK
  setupFSK();

  DBGPRNTST(F("[RTTY] Initializing ... "));

  int16_t state = rtty.begin(RTTY_FREQUENCY, RTTY_SHIFT, RTTY_BAUD, RTTY_MODE, RTTY_STOPBITS);
  if (state == RADIOLIB_ERR_NONE) DBGPRNTLN(F(" success!")); else { DBGPRNT(F(" failed, code: ")); DBGPRNTLN(state); }         
}


//===============================================================================
// Initialize the SX1278 for LoRa
void setupLoRa()
{
  DBGPRNTST(F("[LoRA] Initializing ... "));

  resetRadio();

  int16_t state = radio.begin(LORA_FREQUENCY, LORA_BW, LORA_SPREADFACTOR, LORA_CODERATE, LORA_SYNCWORD, LORA_POWER, LORA_PREAMBLELENGTH, LORA_GAIN);
  if (state == RADIOLIB_ERR_NONE) DBGPRNTLN(F(" success!")); else { DBGPRNT(F(" failed, code: ")); DBGPRNTLN(state); }

#ifdef LORA_EXPLICITMODE
  state = radio.explicitHeader();
#else
  state = radio.implicitHeader(255);
#endif
  if (state != RADIOLIB_ERR_NONE) { DBGPRNTST(F("Setting Lora Explicit / Implicit mode failed, code: ")); DBGPRNTLN(state); }

#ifdef LORA_LDRO
  state = radio.forceLDRO(1);
#else
  state = radio.forceLDRO(0);
#endif
  if (state != RADIOLIB_ERR_NONE) { DBGPRNTST(F("Setting Lora Low Data Rate Optimise failed, code: ")); DBGPRNTLN(state); }

// Set over current protection limit (accepted range is 0 (protection disabled), 45 - 240 mA)
#ifdef LORA_CURRENTLIMIT
  state = radio.setCurrentLimit(LORA_CURRENTLIMIT);
  if (state != RADIOLIB_ERR_NONE) { DBGPRNTST(F("Setting Lora Current Limit failed, code: ")); DBGPRNTLN(state); }
#endif

  state = radio.setCRC(true); // Send a LoRa CRC in all UKHAS LoRa Modes
  if (state != RADIOLIB_ERR_NONE) { DBGPRNTST(F("Setting Lora CRC failed, code: ")); DBGPRNTLN(state); }
}


//===============================================================================
// Initialize the SX1278 for Horus FSK4
void setupFSK4()
{
  // First setup FSK
  setupFSK();

  DBGPRNTST(F("[FSK4] Initializing ... "));
  int16_t state = fsk4.begin(FSK4_FREQ, FSK4_SPACING, FSK4_BAUD);
  if (state == RADIOLIB_ERR_NONE) DBGPRNTLN(F(" success!")); else { DBGPRNT(F(" failed, code: ")); DBGPRNTLN(state); }
}


//===============================================================================
void resetRadio()
{
  // Use for ESP based boards
  /*
  pinMode(PIN_RESET,OUTPUT);
  digitalWrite(PIN_RESET, LOW);
  delay(100);
  digitalWrite(PIN_RESET,HIGH);
  delay(100);
  */
}

//===============================================================================
void interruptSetup(uint32_t len) {
}

//===============================================================================
void setupRadio()
{
  // Setting up the radio
  setupRTTY();
  setupLoRa();
  setupFSK4();

  // https://github.com/jgromes/RadioLib/wiki/Interrupt-Based-Timing
  // ToDO Currently we set interupts up ourselves, change to using RadioLib to do it for us.
  radio.setInterruptSetup(interruptSetup);
}


//===============================================================================
void sendRTTY(const char* TxLine)
{
  // Disable the GPS serial temporarily 
  SERIALGPS.end();

  setupRTTY();

  // Use timer 1 interupt to get best timing possible for each tone
  isr_timer1_start(RTTY_BAUD);

  // Send only idle carrier to let people get their tuning right
  rtty.idle();     
  delay(RTTY_IDLE_TIME);
   
  DBGPRNTST(F("Sending RTTY ... ")); DBGFLUSH(); DBGEND();

  // Send the string 
  int16_t charsSent = rtty.println(TxLine); 

  rtty.standby();

  isr_timer1_stop();
  
  DBGBGN(DBGBAUD); DBGPRNT(F("chars sent: ")); DBGPRNTLN(charsSent);

  // Enable the GPS again.  
  SERIALGPS.begin(GPSBAUD);
}


//===============================================================================
void sendLoRa(const char* TxLine)
{
  setupLoRa();

  DBGPRNTST(F("Sending LoRa ... "));
   
  // Send the string
#ifdef LORA_EXPLICITMODE
  int16_t state = radio.transmit(TxLine); // Send till \0
#else
  int16_t state = radio.transmit((uint8_t*)TxLine, 255); // Send a full 255 chars
#endif
  if (state == RADIOLIB_ERR_NONE) DBGPRNTLN(F("success!")); else { DBGPRNT(F("failed, code: ")); DBGPRNTLN(state); }
}


//===============================================================================
void sendFSK4(uint8_t* codedbuffer, size_t coded_len)
{
  // Disable gps serial temporarily 
  SERIALGPS.end();
  
  setupFSK4();

  DBGPRNTST(F("FSK4 Idle. ")); DBGFLUSH();

  // Use timer 1 interupt to get best timing possible for each tone
  isr_timer1_start(FSK4_BAUD);

  // send out idle condition for 1000 ms
  fsk4.idle();
  delay(FSK4_IDLE_TIME);

  DBGPRNTST(F("Sending FSK4 ... ")); DBGFLUSH(); DBGEND();

  // Send some bytes preamble
  for(int i = 0; i < 8; i++) {
    fsk4.write(0x1B);
  }

  // Send the string
  int16_t charsSent = fsk4.write(codedbuffer, coded_len);
  
  isr_timer1_stop();

  DBGBGN(DBGBAUD); DBGPRNT(F("chars sent: ")); DBGPRNTLN(charsSent);

  // Enable gps serial again.  
  SERIALGPS.begin(GPSBAUD);
}


//===============================================================================
int8_t getRadioTemp()
{
  return radio.getTempRaw();
}


