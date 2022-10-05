// Settings.h

#ifndef SETTINGS_H
#define SETTINGS_H

/***********************************************************************************
* PIN NUMBERS for SX1278
*  
* Change if needed
************************************************************************************/
#define PIN_NSS   PIN_PD4
#define PIN_DIO0  PIN_PD7
#define PIN_RESET PIN_PC6
#define PIN_DIO1  PIN_PD6


/***********************************************************************************
* DEFAULT FSK SETTINGS
*  
* Normally needs no change
************************************************************************************/
#define FSK_FREQUENCY 434.113
#define FSK_BITRATE 100.0
#define FSK_FREQDEV 50.0
#define FSK_RXBANDWIDTH 125.0
#define FSK_POWER 10   // in dBm between 2 and 17. 10 = 10mW (recommended). Sets also RTTY power
#define FSK_PREAMBLELENGTH 16
#define FSK_ENABLEOOK false


/***********************************************************************************
* RTTY SETTINGS
*  
* Change when needed
* Default RTTY setting is: 7,N,2 at 100 Baud.
************************************************************************************/
#define RTTY_ENABLED true            // Set to true if you want RTTY transmissions (You can use Both LoRa and RTTY or only one of the two) 
#define RTTY_PAYLOAD_ID  "RTTY_ID"   // Payload ID for RTTY protocol
#define RTTY_FREQUENCY  434.113      // Can be different from LoRa frequency
#define RTTY_SHIFT 610
#define RTTY_BAUD 100                // Baud rate
#define RTTY_STOPBITS 2
#define RTTY_PREFIX "$$$$$"          
 
// RTTY encoding modes (leave this unchanged)
#define RTTY_ASCII 0                 // 7 data bits 
#define RTTY_ASCII_EXTENDED 1        // 8 data bits
#define RTTY_ITA2  2                 // Baudot 

#define RTTY_REPEATS 1 // number of RTTY transmits during a cycle

// Idle carrier in ms before sending actual RTTY string. 
// Set to a low value (i.e. 1000 or lower) if you have a very frequency stable signal
// Set to a high value (i.e. 5000 or even higher) if you have a hard time to tune the signal
#define RTTY_IDLE_TIME 2500          
 
 
/***********************************************************************************
* LORA SETTINGS
*  
* Change when needed
************************************************************************************/
#define LORA_ENABLED true            // Set to true if you want LoRa transmissions (You can use Both LoRa and RTTY or only one of the two)
#define LORA_PAYLOAD_ID  "LORA-ID"   // Payload ID for LoRa protocol
#define LORA_FREQUENCY  434.562      // Can be different from RTTY frequency
#define LORA_PREFIX "$$"             // Some older LoRa software does not accept a prefix of more than 2x "$"
#define LORA_SYNCWORD 0x12           // for sx1278
// #define LORA_SYNCWORD 0x1424      // for sx1262 (currently not supported)
#define LORA_POWER 10                // in dBm between 2 and 17. 10 = 10mW (recommended)
#define LORA_CURRENTLIMIT 80         // in mA, accepted range is 0 (protection disabled), 45 - 240 mA
#define LORA_PREAMBLELENGTH 8        // length of LoRa preamble in symbols, allowed values range from 6 to 65535
#define LORA_GAIN 0                  // gain of receiver LNA. Can be set to any integer in range 1 to 6 where 1 is the highest gain. Set to 0 to enable automatic gain control (recommended).
#define LORA_MODE 0                  // See HAB LoRa modes below
#define LORA_REPEATS 1               // number of LoRa transmits during a cycle

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


/***********************************************************************************
* TRANSMISSIONS SETTINGS
*  
* Change if needed
************************************************************************************/
#define SENTENCE_LENGTH 100     // Maximum length of telemetry line to send

// Allow time for the GPS to re-acquire a fix when using sleep mode!
// Currently deep sleep is only enabled for ATMEGA328
#define USE_DEEP_SLEEP true     // Put the ATMEGA328 chip to deep sleep while not transmitting. set to true or false.
                                // The tracker will only go to sleep if there are more than 4 satellites visible   
#define TIME_TO_SLEEP  15       // This is the number in seconds out of TX_LOOP_TIME that the CPU is in sleep. Only valid when USE_DEEP_SLEEP = true

#define TX_LOOP_TIME   3       // When USE_DEEP_SLEEP=false: Number in seconds between transmits
                                // When USE_DEEP_SLEEP=true : Time between transmits is TIME_TO_SLEEP+TX_LOOP_TIME+time it takes to transmit the data


/***********************************************************************************
* POWER CONTROL
*  
* Change if needed
************************************************************************************/
// Define up to 5 pins to power sensors from (for example your GPS). Each Arduino pin can source up to 40mA. All together, the pins can source 150-200 mA
// Use a transistor or FET as a switch if you need more power. Or use multiple pins in parallel.
// This will only work when USE_DEEP_SLEEP=true and there is a valid GPS lock.
// Comment out the pins you use for your sensors or leds. 
// Set pin value to a valid value.
#define POWER_PIN1     PIN_PA0
// #define POWER_PIN2     4
// #define POWER_PIN3     -1
// #define POWER_PIN4     -1
// #define POWER_PIN5     -1


/***********************************************************************************
* DEBUG SETTINGS
*  
* Comment DEVMODE out if you experience out-of-memory errors.
************************************************************************************/
#define DEVMODE // Development mode. Uncomment to enable for debugging and see debug info on the serial monitor
#define DBGBAUD 115200
#define SERIALDBG Serial
#define LED PIN_PA3

#ifdef DEVMODE 
  #define DBGBGN(...)   SERIALDBG.begin(__VA_ARGS__)
  #define DBGPRNTST(...)  \
        SERIALDBG.print(millis());     \
        SERIALDBG.print("ms : ");    \
        SERIALDBG.print(__func__); \
        SERIALDBG.print("() : ");      \
        SERIALDBG.print(__LINE__);     \
        SERIALDBG.print(" : ");      \
        SERIALDBG.print(__VA_ARGS__)
  #define DBGPRNTSTLN(...)  \
        SERIALDBG.print(millis());     \
        SERIALDBG.print("ms : ");    \
        SERIALDBG.print(__func__); \
        SERIALDBG.print("() : ");      \
        SERIALDBG.print(__LINE__);     \
        SERIALDBG.print(" : ");      \
        SERIALDBG.println(__VA_ARGS__)
  #define DBGPRNT(...) SERIALDBG.print(__VA_ARGS__)
  #define DBGPRNTLN(...) SERIALDBG.println(__VA_ARGS__)
#else
  #define DBGBGN(...)
  #define DBGPRNTST(...)
  #define DBGPRNTSTLN(...)  
  #define DBGPRNT(...)
  #define DBGPRNTLN(...)  
#endif

       
/***********************************************************************************
* GPS SETTINGS
*  
* Define SERIALGPS if we use a Hardware Serial port for the GPS, 
* or leave it undefined and define Rx and Tx pins for Software serial.
************************************************************************************/
#define GPSBAUD 9600
#define SERIALGPS Serial1

#ifndef SERIALGPS
  // GPS Software Serial pin numbers - free to choose
  // White: 7, Green: 8
  #define GPSRX 7
  #define GPSTX 8
#endif


/***********************************************************************************
* SONDEHUB EXTRA FIELDS SETTINGS
*  
* For displaying extra fields at amateur.sondehub.org, we need to define which fields are
* in the telemetry after the lat, lon, alt fields
* This can be done by adding a specific metadata string after the last telemetry field
* This is supported by the various receivers made by Dave Akerman,
* See: https://www.daveakerman.com/?page_id=2410
* 
* 0  PayloadID
* 1 Counter
* 2 Time
* 3 Latitude
* 4 Longitude
* 5 Altitude
* 6 Satellites
* 7 Speed
* 8 Heading
* 9 Battery Voltage
* A InternalTemperature
* B ExternalTemperature
* C PredictedLatitude
* D PredictedLongitude
* E CutdownStatus
* F LastPacketSNR
* G LastPacketRSSI
* H ReceivedCommandCount
* I-N ExtraFields
* O MaximumAltitude
* P Battery Current
* Q External Temperature 2
* R Pressure
* S Humidity
* T CDA
* U Predicted Landing Speed
* V Time Till Landing
* W Last Command Received
* 
* Our string would be: "0123456A9I"
* You can disable FIELDSTR by undefining it, if you want.
************************************************************************************/
#define USE_FIELDSTR
#define FIELDSTR "0123456A9I"


/***********************************************************************************
* SENSOR SETTINGS
*  
* Change if needed
* 
*  You can connect an external voltage directly to the EXTERNALVOLTAGE_PIN as long as the the pin is rated for that voltage
*  Alteratively, you can use a voltage divider so you can connect a higher voltage, but then you have to calculate the DIVIDER_RATIO yourself
*  
*  Voltage divider schema:
*  
*                          |-----------------
*                          |                |
*                          |               R1            
*                          |                |
*                    +/+ ---                |
*    to external voltage                    |------ To EXTERNALVOLTAGE_PIN
*                    -/- ---                |
*                          |                |
*                          |               R2  
*                          |                |
*                          |----------------------- To Arduino GND
*                          
* DIVIDER_RATIO can be calculated by (R1+R2) / R2                       
*   
* Optionally add a 100nF capacitor between A1 and GND if the measured voltage seems unstable
************************************************************************************/
#define ATMEGA1284P                // Define if using an ATMega1284P
#define USE_EXTERNAL_VOLTAGE true  // Set to true if you want to measure an external voltage on the EXTERNALVOLTAGE_PIN 
#define VCC_OFFSET 0.00            // Offset for error correction in Volts for the internal voltage. Ideally this should be 0.0 but usually is between -0.1 and +0.1 and is chip specific. 
#define EXT_OFFSET 0.00            // Offset for error correction in Volts for the external voltage. Use it to correct errors when necessary.
#define EXTERNALVOLTAGE_PIN PIN_PA4     // Pin to read the external voltage from
#define SAMPLE_RES 1024            // 1024 for Uno, Mini, Nano, Mega, Micro. Leonardo. 4096 for Zero, Due and MKR  
#define DIVIDER_RATIO 4.636        // Leave at 1.00 when using no voltage divider. Set to (R1+R2)/R2 when using a voltage divider. (120+33)/33


/***********************************************************************************
* TELEMETRY COUNTERS
*  
* Uncomment this if you want to reset the counters for LoRa and RTTY set back to 0.
************************************************************************************/
// #define RESET_TRANS_COUNTERS

#endif
