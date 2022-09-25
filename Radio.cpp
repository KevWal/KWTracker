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
SX1278 radio = new Module(PIN_NSS, PIN_DIO0, PIN_DIO1);

// create RTTY client instance using the radio module
RTTYClient rtty(&radio);


//===============================================================================
void SetupRTTY()
{

  // First setup FSK
  SetupFSK();
#if defined(DEVMODE)          
  SERIALDBG.print(F("[RTTY] Initializing ... "));
#endif

  // Struct to hold RTTY settings
  struct TRTTYSettings
  {
    float Frequency = RTTY_FREQUENCY;   // Base frequency
    uint32_t Shift = RTTY_SHIFT;        // RTTY shift
    uint16_t Baud = RTTY_BAUD;          // Baud rate
    uint8_t Encoding = RTTY_ASCII;   // Encoding (ASCII = 7 bits)
    uint8_t StopBits = RTTY_STOPBITS;   // Number of stopbits 
  } RTTYSettings;

  int16_t state = rtty.begin(RTTYSettings.Frequency,
                     RTTYSettings.Shift,
                     RTTYSettings.Baud,
                     RTTYSettings.Encoding,
                     RTTYSettings.StopBits  );
                     
#if defined(DEVMODE)            
  if(state == ERR_NONE) 
  {          
    SERIALDBG.println(F("success!"));
  } else 
  {
    SERIALDBG.print(F("failed, code "));
    SERIALDBG.println(state);
  }
#endif
}


//===============================================================================
void SetupFSK()
{
  // Reset the radio
  ResetRadio();
  // Initialize the SX1278
#if defined(DEVMODE)  
  SERIALDBG.print(F("[SX1278] Initializing ... "));
#endif

 // int16_t state = radio.beginFSK();
 
  // Struct to hold FSK settings
  struct TFSKSettings
  {
    float Frequency = FSK_FREQUENCY;
    float BitRate = FSK_BITRATE; 
    float FreqDev = FSK_FREQDEV;
    float RXBandwidth = FSK_RXBANDWIDTH;
    int8_t Power = FSK_POWER;                  // in dbM range 2 - 17
    uint16_t PreambleLength = FSK_PREAMBLELENGTH;
    bool EnableOOK = FSK_ENABLEOOK;
    float dataShaping = FSK_DATASHAPING;
  } FSKSettings;

  int16_t state = radio.beginFSK(FSKSettings.Frequency,
                               FSKSettings.BitRate,
                               FSKSettings.FreqDev,
                               FSKSettings.RXBandwidth,
                               FSKSettings.Power,
                               FSKSettings.PreambleLength,
                               FSKSettings.EnableOOK);


#if defined(DEVMODE) 
  if(state == ERR_NONE) 
  {   
    SERIALDBG.println(F("success!"));
  } 
  else 
  {
    SERIALDBG.print(F("failed, code "));
    SERIALDBG.println(state);
  }
#endif
}


//===============================================================================
void SetupLoRa()
{
  // Initialize the SX1278
#if defined(DEVMODE)  
  SERIALDBG.print(F("[LoRA] Initializing ... "));
#endif

  ResetRadio();

  // Struct to hold LoRA settings
  struct TLoRaSettings
  {
    float Frequency = LORA_FREQUENCY;
    float Bandwidth = LORA_BANDWIDTH;
    uint8_t SpreadFactor = LORA_SPREADFACTOR;
    uint8_t CodeRate = LORA_CODERATE;
    uint8_t SyncWord = LORA_SYNCWORD;
    uint8_t Power = LORA_POWER;
    uint8_t CurrentLimit = LORA_CURRENTLIMIT;
    uint16_t PreambleLength =  LORA_PREAMBLELENGTH;
    uint8_t Gain = LORA_GAIN;
  } LoRaSettings;

  // First setup the mode
  // 0 = (normal for telemetry)  Explicit mode, Error coding 4:8, Bandwidth 20.8kHz, SF 11, Low data rate optimize on - NOT IMPLEMENTED YET
  // 1 = (normal for SSDV)       Implicit mode, Error coding 4:5, Bandwidth 20.8kHz,  SF 6, Low data rate optimize off - NOT IMPLEMENTED YET
  
  // 2 = (normal for repeater)   Explicit mode, Error coding 4:8, Bandwidth 62.5kHz,  SF 8, Low data rate optimize off - Should work correctly
  // 3 = (normal for fast SSDV)  Explicit mode, Error coding 4:6, Bandwidth 250kHz,   SF 7, Low data rate optimize off - Should work correctly
  switch (LORA_MODE)
  {
    case 0: 
      LoRaSettings.CodeRate = 8;
      LoRaSettings.Bandwidth = 20.8;
      LoRaSettings.SpreadFactor = 11;
      break;   

    case 1:
      LoRaSettings.CodeRate = 5;
      LoRaSettings.Bandwidth = 20.8;      
      LoRaSettings.SpreadFactor = 6;      
      break;   
    
    case 2:
      LoRaSettings.CodeRate = 8;
      LoRaSettings.Bandwidth = 62.5;      
      LoRaSettings.SpreadFactor = 8;      
      break;   

    case 3:
      LoRaSettings.CodeRate = 6;
      LoRaSettings.Bandwidth = 250;      
      LoRaSettings.SpreadFactor = 7;            
      break;   
  }
  
  int16_t state = radio.begin
  (
    LoRaSettings.Frequency,
    LoRaSettings.Bandwidth,
    LoRaSettings.SpreadFactor,
    LoRaSettings.CodeRate,
    LoRaSettings.SyncWord,
    LoRaSettings.Power,
    LoRaSettings.PreambleLength, 
    LoRaSettings.Gain
  );
  
#if defined(DEVMODE)
  if(state == ERR_NONE) 
  {    
    SERIALDBG.println(F("success!"));  
  } 
  else 
  {
    SERIALDBG.print(F("failed, code "));
    SERIALDBG.println(state);
  }
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
   
#if defined(DEVMODE)   
  SERIALDBG.print(F("Sending RTTY: "));
  SERIALDBG.println(TxLine);
#endif   
   
  // Send the string 
  int state = rtty.println(TxLine); 

#if defined(DEVMODE)
  if(state == ERR_NONE) 
  {    
    SERIALDBG.println(F("success!"));  
  } 
  else 
  {
    SERIALDBG.print(F("failed, code "));
    SERIALDBG.println(state);
  }
#endif

  // Enable the GPS again.  
  SERIALGPS.begin(GPSBAUD);
}


//===============================================================================
void sendLoRa(String TxLine)
{
  SetupLoRa();

#if defined(DEVMODE)      
  SERIALDBG.print(F("Sending LoRa: "));
  SERIALDBG.println(TxLine);
#endif
   
  // Send the string
  int state = radio.transmit(TxLine); 

#if defined(DEVMODE)
  if(state == ERR_NONE) 
  {    
    SERIALDBG.println(F("success!"));  
  } 
  else 
  {
    SERIALDBG.print(F("failed, code "));
    SERIALDBG.println(state);
  }
#endif

}
