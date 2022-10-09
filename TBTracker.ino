// TBTracker.ino

/***********************************************************************************
 *  FIRST THING YOU NEED TO DO IS ADJUST THE SETTINGS IN Settings.h and have FUN!
 ***********************************************************************************/

/***********************************************************************************
 * LoRa and RTTY tracker for Arduino and SX1278
 * Specially designed for the cheap TTGO T-Deer board
 * a TTGO T-Deer board is an Arduino Mini with a SX1278 LoRa chip on board
 * 
 * by Roel Kroes
 * roel@kroes.com
 *
 *  Extra libraries needed:
 *  https://github.com/jgromes/RadioLib (Radiolib)
 *  https://github.com/mikalhart/TinyGPSPlus (tinyGPS++)
 *
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
long RTTYCounter = 0;
long LoRaCounter = 0;
unsigned long previousTX = 0;
volatile bool watchdogActivated = true;
volatile int sleepIterations = 0;
TGPS UGPS;    // Structure to hold GPS data

// If SERIALGPS is defined, then we are using a Hardware serial port for the GPS and dont need Software Serial
#ifndef SERIALGPS
  #include <SoftwareSerial.h>
  SoftwareSerial SERIALGPS(GPSRX, GPSTX);
#endif


//============================================================================
void setup()
{
  // Setup Serial for debugging
  DBGBGN(DBGBAUD);

  // Setup the UBlox GPS
  SERIALGPS.begin(GPSBAUD);

#if defined(RESET_TRANS_COUNTERS)
  resetTransmissionCounters();
#endif

  // Setup the Radio
  resetRadio(); 
  setupRadio();  

  // Setup power control
  setupPowerPins();
  enablePowerPins();

  // Setup LED
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
}


//============================================================================
void loop()
{
  // Watchdog should have been fired before doing anything 
  if (watchdogActivated)
  {
    // Reset the watchdog and the sleep timer
    watchdogActivated = false;
     
    // Get data from the GPS
    smartDelay(2500);
    checkGPS(); 

    unsigned long currentMillis = millis();
    // Process a non blocking timed Transmit & Sleep loop
    if (currentMillis - previousTX >= ((unsigned long)TX_LOOP_TIME*(unsigned long)1000))
    {

      if (UGPS.Satellites > 5) // If we have a reasonable GPS signal, reduce power usage by turning off GPS before we try and transmit.
        disablePowerPins();

      previousTX = currentMillis;
    
      // Send RTTY
      if (RTTY_ENABLED)
      { 
        for (int i=1; i <= RTTY_REPEATS; i++)
        {
          RTTYCounter = EEPROMReadlong(0x00);
          createRTTYTXLine(RTTY_PAYLOAD_ID, RTTYCounter++, RTTY_PREFIX);
          sendRTTY(Sentence); 
          // Write the RTTY counter to EEPROM at address 0x00
          EEPROMWritelong(0x00, RTTYCounter);
        }
      } // if (RTTY_ENABLED)

      // Delay in milliseconds between rtty and lora. You can change this
      delay(1000);
     
      // Send LoRa 
      if (LORA_ENABLED)
      { 
        for (int i=1; i <= LORA_REPEATS; i++)
        {
          LoRaCounter = EEPROMReadlong(0x04);
          createLoRaTXLine(LORA_PAYLOAD_ID, LoRaCounter++, LORA_PREFIX);
          sendLoRa(Sentence); 
          // Write the LoRa counter to EEPROM at address 0x04
          EEPROMWritelong(0x04, LoRaCounter);
        }
      } // if (LORA_ENABLED)

      // Go to sleep after transmissions
      if (USE_DEEP_SLEEP)
      {
        DBGPRNTST(F("Start sleep "));
        DBGFLUSH();
        sleepIterations = 0;    
        while (sleepIterations < TIME_TO_SLEEP)
        {
          mySleep(); // WDT iterates sleepIterations
          DBGPRNT(".>. "); DBGFLUSH();
        }
        DBGPRNTLN(F("awake!"));
        previousTX = millis();
      } // if (USE_DEEP_SLEEP)

      enablePowerPins();  // Turn GPS power back on
      UGPS.FlightMode = UNKNOWN; // GPS mode now unknown

    } // if (currentMillis - previousTX >= ((unsigned long)TX_LOOP_TIME*(unsigned long)1000))

    watchdogActivated = true; 
  } // if (watchdogActivated)

} // loop
