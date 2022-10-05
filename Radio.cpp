// Radio.cpp

/***********************************************************************************
* #includes
*
* #include user defined headers, followed by 3rd party library headers, then standard 
* library headers, with the headers in each section sorted alphabetically.
************************************************************************************/
#include "Radio.h"
#include "TBTracker.h"
#include "Settings.h"

#include <RadioLib.h>

// Change 'SX1278' in the line below to 'SX1276' if you have a SX1276 module.
SX1278 radio = new Module(PIN_NSS, PIN_DIO0, PIN_RESET, PIN_DIO1);

// create RTTY client instance using the radio module
RTTYClient rtty(&radio);


//===============================================================================
void SetupRTTY()
{
  // First setup FSK
  SetupFSK();

  DBGPRNTST(F("[RTTY] Initializing ... "));

  int16_t state = rtty.begin(RTTY_FREQUENCY, RTTY_SHIFT, RTTY_BAUD, RTTY_ASCII, RTTY_STOPBITS);
  if (state == RADIOLIB_ERR_NONE) DBGPRNTLN(F(" success!")); else { DBGPRNT(F(" failed, code: ")); DBGPRNTLN(state); }         
}


//===============================================================================
void SetupFSK()
{
  // Reset the radio
  ResetRadio();

  // Initialize the SX1278
  DBGPRNTST(F("[SX1278] Initializing ... "));

  int16_t state = radio.beginFSK(FSK_FREQUENCY, FSK_BITRATE, FSK_FREQDEV, FSK_RXBANDWIDTH, FSK_POWER, FSK_PREAMBLELENGTH, FSK_ENABLEOOK);
  if (state == RADIOLIB_ERR_NONE) DBGPRNTLN(F(" success!")); else { DBGPRNT(F(" failed, code: ")); DBGPRNTLN(state); }
}


//===============================================================================
// Initialize the SX1278 for LoRa
void SetupLoRa()
{
  DBGPRNTST(F("[LoRA] Initializing ... "));

  ResetRadio();

// HAB LoRa Modes:
// Num; ImplicitOrExplicit; ErrorCoding; Bandwidth; SpreadingFactor; LowDataRateOptimize; BaudRate; Description
// 0: EXPLICIT_MODE, ERROR_CODING_4_8, BANDWIDTH_20K8, SPREADING_11, 1,    60, Telemetry - Normal mode for telemetry
// 1: IMPLICIT_MODE, ERROR_CODING_4_5, BANDWIDTH_20K8, SPREADING_6,  0,  1400, SSDV - Normal mode for SSDV
// 2: EXPLICIT_MODE, ERROR_CODING_4_8, BANDWIDTH_62K5, SPREADING_8,  0,  2000, Repeater - Normal mode for repeater network	
// 3: EXPLICIT_MODE, ERROR_CODING_4_6, BANDWIDTH_250K, SPREADING_7,  0,  8000, Turbo - Normal mode for high speed images in 868MHz band
// 4: IMPLICIT_MODE, ERROR_CODING_4_5, BANDWIDTH_250K, SPREADING_6,  0, 16828, TurboX - Fastest mode within IR2030 in 868MHz band
// 5: EXPLICIT_MODE, ERROR_CODING_4_8, BANDWIDTH_41K7, SPREADING_11, 0,   200, Calling - Calling mode
// 6: EXPLICIT_MODE, ERROR_CODING_4_5, BANDWIDTH_20K8, SPREADING_7,  0,  2800, Uplink - Uplink explicit mode (variable length)
// 7: IMPLICIT_MODE, ERROR_CODING_4_5, BANDWIDTH_41K7, SPREADING_6,  0,  2800, Uplink - Uplink mode for 868
// 8: EXPLICIT_MODE, ERROR_CODING_4_5, BANDWIDTH_20K8, SPREADING_7,  0,   910, Telnet - Telnet-style comms with HAB on 434
// 9: IMPLICIT_MODE, ERROR_CODING_4_5, BANDWIDTH_62K5, SPREADING_6,  0,  4500, SSDV Repeater - Fast (SSDV) repeater network

#if LORA_MODE == 0
  #define LORA_EXPLICITMODE
  #define LORA_CODERATE 8
  #define LORA_BW 20.8
  #define LORA_SPREADFACTOR 11
  #define LORA_LDRO
#endif

#if LORA_MODE == 1
  #define LORA_CODERATE 5
  #define LORA_BW 20.8
  #define LORA_SPREADFACTOR 6
#endif

#if LORA_MODE == 2
  #define LORA_EXPLICITMODE
  #define LORA_CODERATE 8
  #define LORA_BW 62.5
  #define LORA_SPREADFACTOR 8
#endif

  int16_t state = radio.begin(LORA_FREQUENCY, LORA_BW, LORA_SPREADFACTOR, LORA_CODERATE, LORA_SYNCWORD, LORA_POWER, LORA_PREAMBLELENGTH, LORA_GAIN);
  if (state == RADIOLIB_ERR_NONE) DBGPRNTLN(F(" success!")); else { DBGPRNT(F(" failed, code: ")); DBGPRNTLN(state); }

#ifdef LORA_EXPLICITMODE
  state = radio.explicitHeader();
#else
  state = raido.implicitHeader(255);
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
}


//===============================================================================
void ResetRadio()
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
void SetupRadio()
{
  // Setting up the radio
  SetupRTTY();
  SetupLoRa();
}


//===============================================================================
void sendRTTY(String TxLine)
{
  // Disable the GPS serial temporarily 
  SERIALGPS.end();

  SetupRTTY();
   
  // Send only idle carrier to let people get their tuning right
  rtty.idle();     
  delay(RTTY_IDLE_TIME);
   
  DBGPRNTST(F("Sending RTTY ... "));
   
  // Send the string 
  int16_t charsSent = rtty.println(TxLine); 
  
  DBGPRNT(F("chars sent: ")); DBGPRNTLN(charsSent);

  // Enable the GPS again.  
  SERIALGPS.begin(GPSBAUD);
}


//===============================================================================
void sendLoRa(String TxLine)
{
  SetupLoRa();

  DBGPRNTST(F("Sending LoRa ... "));
   
  // Send the string
  int16_t state = radio.transmit(TxLine);
  if (state == RADIOLIB_ERR_NONE) DBGPRNTLN(F("success!")); else { DBGPRNT(F("failed, code: ")); DBGPRNTLN(state); }
}
