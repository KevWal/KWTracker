// TBTracker.ino

/***********************************************************************************
 *  FIRST THING YOU NEED TO DO IS ADJUST THE SETTINGS IN Settings.h
 *  
 *  Have FUN!
 ***********************************************************************************/

/***********************************************************************************
 * LoRa and RTTY tracker for Arduino and SX1278
 * Specially designed for the cheap TTGO T-Deer board
 * a TTGO T-Deer board is an Arduino Mini with a SX1278 LoRa chip on board
 * 
 * by Roel Kroes
 * roel@kroes.com
 *
 *  
 *  To run this, you need a TTGO T-Deer board or:
 *  1 x Arduino (or compatible Arduino board)
 *  1 x SX1278 LoRa chip (or compatible LoRa chip)
 *  1 x BN220 GPS (Ublox compatible / 9600 Baud) or any compatible GPS device
 *    
 *  BN22 <> Arduino
 *  ----------------------  
 *  VCC -> 3.3V (red wire)
 *  GND -> GND (black wire)
 *  RX -> D7 (white wire)
 *  TX -> D8 (green wire)
 *  
 *  (Example for Arduino Nano)
 *  SX1278 <-> Arduino Nano
 *  ----------------------
 *  MISO   -> D12
 *  SCK    -> D13
 *  RST    -> Not connected
 *  GND    -> GND
 *  DI00   -> D2
 *  MOSI   -> D11
 *  NSS    -> D10 
 *  3V3    -> 3V3
 *  
 *  Extra libraries needed:
 *  https://github.com/jgromes/RadioLib (Radiolib)
 *  https://github.com/mikalhart/TinyGPSPlus (tinyGPS++)
 *  
 *  For payload information and how to get your Payload on the map, see the file Misc.ini from this sketch
 ************************************************************************************/

/***********************************************************************************
* #includes
*  
* Normally no change necessary
*
* #include user defined headers, followed by 3rd party library headers, then standard 
* library headers, with the headers in each section sorted alphabetically.
************************************************************************************/
#include "ADC.h"
#include "GPS.h"
#include "Misc.h"
#include "Radio.h"
#include "Sleep.h"
#include "Settings.h"
#include "TBTracker.h"

#include <Arduino.h>

/***********************************************************************************
* GLOBALS
*  
* Normally no change necessary
************************************************************************************/
char Sentence[SENTENCE_LENGTH];
long RTTYCounter=0;
long LoRaCounter=0;
unsigned long previousTX = 0;
volatile bool watchdogActivated = true;
volatile int sleepIterations = 0;
TGPS UGPS;    // Structure to hold GPS data

//If SerialGPS is defined then we are using a Hardware serial port and dont need Software Serial
#ifndef SERIALGPS
  #include <SoftwareSerial.h>
  SoftwareSerial SERIALGPS(GPSRX, GPSTX);
#endif




//============================================================================
void setup()
{
  // Setup Serial for debugging
  SERIALDBG.begin(DBGBAUD);
  // Setup the UBlox GPS
  SERIALGPS.begin(GPSBAUD);

#if defined(RESET_TRANS_COUNTERS)
  Reset_Transmission_Counters();
#endif

  // Setup the Radio
  ResetRadio(); 
  SetupRadio();  
  setup_PowerPins();
  enable_PowerPins();
}


//============================================================================
void loop()
{
   // Watchdog should have been fired before doing anything 
  if (watchdogActivated)
  {
     // REset the watchdog and the sleep timer
     watchdogActivated = false;
     
     unsigned long currentMillis = millis();
   
     // Get data from the GPS
     CheckGPS(); 
     smartDelay(1000);
 
     // Process a non blocking timed loop
     if (currentMillis - previousTX >= ((unsigned long)TX_LOOP_TIME*(unsigned long)1000))
     {
       // Telemetry loop  
       previousTX = currentMillis;
     
       // Send RTTY
       if (RTTY_ENABLED)
       { 
          for (int i=1; i <= RTTY_REPEATS; i++)
          {
            RTTYCounter = EEPROMReadlong(0x00);
            CreateTXLine(RTTY_PAYLOAD_ID, RTTYCounter++, RTTY_PREFIX);
            sendRTTY(Sentence); 
            // Write the RTTY counter to EEPROM at address 0x00
            EEPROMWritelong(0x00, RTTYCounter);
          }
       }

       // Delay in milliseconds between rtty and lora. You can change this
       delay(1000);
     
       // Send LoRa 
       if (LORA_ENABLED)
       { 
          for (int i=1; i <= LORA_REPEATS; i++)
          {
            LoRaCounter = EEPROMReadlong(0x04);
            CreateTXLine(LORA_PAYLOAD_ID, LoRaCounter++, LORA_PREFIX);
            sendLoRa(Sentence); 
            // Write the LoRa counter to EEPROM at address 0x04
            EEPROMWritelong(0x04, LoRaCounter);
          }
       }
       // Goto to sleep after transmissions
       if (USE_DEEP_SLEEP && UGPS.Satellites > 4)
       {
#if defined(DEVMODE)        
         SERIALDBG.println("Start sleep..");
#endif
         // Set all defined power pins to low
         disable_PowerPins();
         SERIALDBG.flush();
         sleepIterations = 0;    
         while (sleepIterations < TIME_TO_SLEEP)
         {
           my_Sleep();
         }
#if defined(DEVMODE)                 
        SERIALDBG.println("Awake!");
#endif       
        // Set all defined power pins to high
        enable_PowerPins();
        previousTX = millis();
       }
    }
    watchdogActivated = true; 
  }
}
