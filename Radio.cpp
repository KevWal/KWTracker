// Radio.cpp

/***********************************************************************************
* #includes
*
* #include user defined headers, followed by 3rd party library headers, then standard 
* library headers, with the headers in each section sorted alphabetically.
************************************************************************************/
#include "Radio.h"
#include "KWTracker.h"
#include "Settings.h"
#include "Horus_L1.h"

#include <RadioLib.h>

// Change 'SX1278' in the line below to 'SX1276' if you have a SX1276 module.
SX1278 radio = new Module(PIN_NSS, PIN_DIO0, PIN_RESET, PIN_DIO1);

// create RTTY client instance using the radio module
RTTYClient rtty(&radio);


//===============================================================================
void setupFSK()
{
  // Reset the radio
  resetRadio();

  // Initialize the SX1278
  DBGPRNTST(F("[SX1278] Initializing ... "));

  int16_t state = radio.beginFSK(FSK_FREQUENCY, FSK_BITRATE, FSK_FREQDEV, FSK_RXBANDWIDTH, FSK_POWER, FSK_PREAMBLELENGTH, FSK_ENABLEOOK);
  if (state == RADIOLIB_ERR_NONE) DBGPRNTLN(F(" success!")); else { DBGPRNT(F(" failed, code: ")); DBGPRNTLN(state); }
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

  int16_t state = fsk4_setup(&radio, FSK4_FREQ, FSK4_SPACING, FSK4_BAUD);
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
void setupRadio()
{
  // Setting up the radio
  setupRTTY();
  setupLoRa();
  setupFSK4();
}


//===============================================================================
void sendRTTY(const char* TxLine)
{
  // Disable the GPS serial temporarily 
  SERIALGPS.end();

  setupRTTY();
   
  // Send only idle carrier to let people get their tuning right
  rtty.idle();     
  delay(RTTY_IDLE_TIME);
   
  DBGPRNTST(F("Sending RTTY ... ")); DBGFLUSH(); DBGEND();
   
  // Send the string 
  int16_t charsSent = rtty.println(TxLine); 
  
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
  // Disable the GPS serial temporarily 
  SERIALGPS.end();

  setupFSK4();

  // send out idle condition for 1000 ms
  fsk4_idle(&radio);
  delay(FSK4_IDLE_TIME);

  DBGPRNTST(F("Sending FSK4 ... ")); DBGFLUSH();
   
  // Send the string
  //int16_t state = XXX(TxLine); // Send till \0
  fsk4_preamble(&radio, 8);
  fsk4_write(&radio, codedbuffer, coded_len);
  //if (state == RADIOLIB_ERR_NONE) DBGPRNTLN(F("success!")); else { DBGPRNT(F("failed, code: ")); DBGPRNTLN(state); }
  DBGPRNTLN(F("done."));

  // Enable the GPS again.  
  SERIALGPS.begin(GPSBAUD);
}


//===============================================================================
int8_t getRadioTemp()
{
  return radio.getTempRaw();
}


