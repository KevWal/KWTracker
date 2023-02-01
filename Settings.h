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
* Normally needs no change, just base settings, not used for RTTY, Lora nor FSK4 modes apart from POWER
************************************************************************************/
#define FSK_FREQUENCY 434.0   // FSK Frequency in Mhz, Default 434.0, not used
#define FSK_BITRATE 4.8       // FSK Bit rate in kbps (kilobits per second). Default 4.8. Allowed values range from 1.2 to 300.0 kbps, not used
#define FSK_FREQDEV 5.0       // FSK Frequency deviation kHz. Default 5.0. Allowed values range from 0.6 to 200.0 kHz, not used
                              // Note that the allowed range changes based on bit rate setting, so that the condition FreqDev + BitRate/2 <= 250 kHz is always met
#define FSK_RXBANDWIDTH 125.0 // Default 125.0, not used
#define FSK_PREAMBLELENGTH 16 // Default 16, not used
#define FSK_ENABLEOOK false   // Use OOK (On Off Keying) modulation instead of FSK. Default False

#define FSK_POWER 10          // FSK Power in dBm, Default 10, valid between 2 and 17. 10 = 10mW (recommended). ***Sets RTTY & FSK4 power
#define FSK_CURRENTLIMIT 80   // in mA, accepted range is 0 (protection disabled), 45 - 240 mA


/***********************************************************************************
* RTTY SETTINGS
*  
* Change when needed
* Default RTTY setting is: 7,N,2 at 100 Baud.
************************************************************************************/
#define RTTY_ENABLED false             // Set to true if you want RTTY transmissions (You can use Both LoRa and RTTY or only one of the two) 
#define RTTY_PAYLOAD_ID  "KW-BCK1"    // Payload ID for RTTY protocol
#define RTTY_FREQUENCY  434.113       // Can be different from LoRa frequency
#define RTTY_SHIFT 610                //  NOTE: RTTY frequency shift will be rounded to the nearest multiple of module frequency step size. SX127x/RFM9x - 61 Hz
#define RTTY_BAUD 50                  // Baud rate
#define RTTY_STOPBITS 2
#define RTTY_PREFIX "$$$$$"          

#define RTTY_MODE RADIOLIB_ASCII_EXTENDED // RTTY encoding mode (RADIOLIB_ITA2 (5bits), RADIOLIB_ASCII_EXTENDED (8bits) or RADIOLIB_ASCII (7bits))

#define RTTY_REPEATS 3 // number of RTTY transmits during a cycle

// Idle carrier in ms before sending actual RTTY string. 
// Set to a low value (i.e. 1000 or lower) if you have a very frequency stable signal
// Set to a high value (i.e. 5000 or even higher) if you have a hard time to tune the signal
#define RTTY_IDLE_TIME 2000          
 
 
/***********************************************************************************
* LORA SETTINGS
*  
* Change when needed
************************************************************************************/
#define LORA_ENABLED true            // Set to true if you want LoRa transmissions (You can use Both LoRa and RTTY or only one of the two)
#define LORA_PAYLOAD_ID "KW-BCK2"   // Payload ID for LoRa protocol
//#define LORA_FREQUENCY 434.562      // Can be different from RTTY frequency
//#define LORA_FREQUENCY 433.650      // Can be different from RTTY frequency
//#define LORA_FREQUENCY 433.662      // Can be different from RTTY frequency
#define LORA_FREQUENCY 433.090      // Can be different from RTTY frequency
#define LORA_PREFIX "$$"             // Some older LoRa software does not accept a prefix of more than 2x "$"
#define LORA_SYNCWORD 0x12           // for sx1278
// #define LORA_SYNCWORD 0x1424      // for sx1262 (currently not supported)
#define LORA_POWER 10                // in dBm between 2 and 17. 10 = 10mW (recommended)
#define LORA_CURRENTLIMIT 80         // in mA, accepted range is 0 (protection disabled), 45 - 240 mA
#define LORA_PREAMBLELENGTH 8        // length of LoRa preamble in symbols, allowed values range from 6 to 65535
#define LORA_GAIN 0                  // gain of receiver LNA. Can be set to any integer in range 1 to 6 where 1 is the highest gain. Set to 0 to enable automatic gain control (recommended).

//#define LORA_MODE 5                // See HAB LoRa modes below
#define LORA_MODE 2                  // See HAB LoRa modes below
#define LORA_REPEATS 3              // number of LoRa transmits during a cycle

// HAB LoRa Modes:
// Narrower bandwidth improves range, higher spreading factor increases reslience, higher error coding rate increases Forward Error Correction
// Num; ImplicitOrExplicit; ErrorCoding; Bandwidth; SpreadingFactor; LowDataRateOptimize; BaudRate; Description
// 0: EXPLICIT_MODE, ERROR_CODING_4_8, BANDWIDTH_20K8, SPREADING_11, 1,    60, Telemetry - Normal mode for telemetry
// 1: IMPLICIT_MODE, ERROR_CODING_4_5, BANDWIDTH_20K8, SPREADING_6,  0,  1400, SSDV - Normal mode for SSDV
// 2: EXPLICIT_MODE, ERROR_CODING_4_8, BANDWIDTH_62K5, SPREADING_8,  0,  2000, Repeater - Normal mode for repeater network	
// 3: EXPLICIT_MODE, ERROR_CODING_4_6, BANDWIDTH_250K, SPREADING_7,  0,  8000, Turbo - Normal mode for high speed images in 868MHz band
// 4: IMPLICIT_MODE, ERROR_CODING_4_5, BANDWIDTH_250K, SPREADING_6,  0, 16828, TurboX - Fastest mode within IR2030 in 868MHz band
// 5: EXPLICIT_MODE, ERROR_CODING_4_8, BANDWIDTH_41K7, SPREADING_11, 0,   200, Calling - Calling mode
// 6: EXPLICIT_MODE, ERROR_CODING_4_5, BANDWIDTH_20K8, SPREADING_7,  0,  2800, Uplink - Uplink explicit mode (variable length) (note commented out in some versions of lora-gateway)
// 7: IMPLICIT_MODE, ERROR_CODING_4_5, BANDWIDTH_41K7, SPREADING_6,  0,  2800, Uplink - Uplink mode for 868
// 8: EXPLICIT_MODE, ERROR_CODING_4_5, BANDWIDTH_20K8, SPREADING_7,  0,   910, Telnet - Telnet-style comms with HAB on 434
// 9: IMPLICIT_MODE, ERROR_CODING_4_5, BANDWIDTH_62K5, SPREADING_6,  0,  4500, SSDV Repeater - Fast (SSDV) repeater network

// For 433Mhz ISM in the UK comply with:
// IR2030/1/10 433.05-434.79 MHz 10 mW e.r.p. Duty cycle limit 10% or;
// IR2030/1/11 433.05-434.79 MHz 1mW erp (no duty cycle limit);
// IR2030/1/12 433.04-434.79 MHz 10 mW e.r.p. Channel Spacing <= 25 kHz


/***********************************************************************************
* HORUS FSK4 Settings
*  
* Change when needed
************************************************************************************/
#define FSK4_ENABLED false            // Set to true if you want FSK4 transmissions
#define FSK4_PAYLOAD_ID 333        // Int16 Payload ID for FSK4 protocol, 256 = test for v2 mode
#define FSK4_FREQ       434.340
#define FSK4_BAUD       100
#define FSK4_SPACING    244          // 270 results in a shift of 244 Hz due to the PLL Resolution of the SX127x

#define FSK4_IDLE_TIME 2000         // Idle carrier in ms before sending actual FSK4 string.

#define FSK4_REPEATS 3              // number of FSK4 transmits during a cycle


/***********************************************************************************
* TRANSMISSIONS SETTINGS
*  
* Change if needed
************************************************************************************/
#define SENTENCE_LENGTH 256     // Maximum length of telemetry line to send, needs to be 256 when using an implicit mode

// Allow time for the GPS to re-acquire a fix when using sleep mode!
#define USE_DEEP_SLEEP true     // Put the ATMEGA chip to deep sleep while not transmitting. set to true or false.
                                // The tracker will only go to sleep if there are more than X satellites visible   
#define TIME_TO_SLEEP  10      // This is the number in seconds out of TX_LOOP_TIME that the CPU is in sleep. Only valid when USE_DEEP_SLEEP = true

#define TX_LOOP_TIME   30       // When USE_DEEP_SLEEP=false: Number in seconds between transmits
                                // When USE_DEEP_SLEEP=true : Time between transmits is TIME_TO_SLEEP+TX_LOOP_TIME+time it takes to transmit the data
                                // When USE_DEEP_SLEEP=true TX_LOOP_TIME needs to be long enough to regain a GPS fix.


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
// #define POWER_PIN2     -1
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
#define LED PIN_PA3  // Basic Tracker LED is PA3

// Port and Pin manually set in code!
//#define DEVTIMING PIN_PB4 // Enable to test FSK4 Timing
       
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
* SONDEHUB EXTRA FIELDS SETTINGS (only used by LoRa)
*  
* For displaying extra fields at amateur.sondehub.org, we need to define which fields are
* in the telemetry after the lat, lon, alt fields
* This can be done by adding a specific metadata string after the last telemetry field
* This is supported by the various receivers made by Dave Akerman,
* See: https://www.daveakerman.com/?page_id=2410
* 
* 0 PayloadID, 1 Counter, 2 Time
* 3 Latitude, 4 Longitude
* 5 Altitude, 6 Satellites
* 7 Speed, 8 Heading
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
************************************************************************************/
#define USE_FIELDSTR
//#define FIELDSTR "0123456AI9"
// Making LoRa sentance a better fit to TinyGS.com parsing
#define FIELDSTR "01234568A9I"

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
//#define RESET_TRANS_COUNTERS


/***********************************************************************************
* #define Macros
*  
* No change normally needed
************************************************************************************/
#ifdef DEVMODE 
  #define DBGBGN(...)   SERIALDBG.begin(__VA_ARGS__)
  #define DBGEND(...)    SERIALDBG.end(__VA_ARGS__)
  #define DBGFLUSH(...)  SERIALDBG.flush()
  #define DBGPRNTST(...)  \
        SERIALDBG.print(__func__); \
        SERIALDBG.print("() : ");      \
        SERIALDBG.print(__LINE__);     \
        SERIALDBG.print(" : ");      \
        SERIALDBG.print(__VA_ARGS__)
  #define DBGPRNTSTLN(...)  \
        SERIALDBG.print(__func__); \
        SERIALDBG.print("() : ");      \
        SERIALDBG.print(__LINE__);     \
        SERIALDBG.print(" : ");      \
        SERIALDBG.println(__VA_ARGS__)
  #define DBGPRNT(...) SERIALDBG.print(__VA_ARGS__)
  #define DBGPRNTLN(...) SERIALDBG.println(__VA_ARGS__)
#else
  #define DBGBGN(...)
  #define DBGEND(...)
  #define DBGFLUSH(...)
  #define DBGPRNTST(...)
  #define DBGPRNTSTLN(...)  
  #define DBGPRNT(...)
  #define DBGPRNTLN(...)  
#endif

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

#if LORA_MODE == 3
  #define LORA_EXPLICITMODE
  #define LORA_CODERATE 6
  #define LORA_BW 250
  #define LORA_SPREADFACTOR 7
#endif

#if LORA_MODE == 4
  #define LORA_CODERATE 5
  #define LORA_BW 250
  #define LORA_SPREADFACTOR 6
#endif

#if LORA_MODE == 5
  #define LORA_EXPLICITMODE
  #define LORA_CODERATE 8
  #define LORA_BW 41.7
  #define LORA_SPREADFACTOR 11
#endif

#if LORA_MODE == 6
  #define LORA_EXPLICITMODE
  #define LORA_CODERATE 5
  #define LORA_BW 20.8
  #define LORA_SPREADFACTOR 7
#endif

#if LORA_MODE == 7
  #define LORA_CODERATE 5
  #define LORA_BW 41.7
  #define LORA_SPREADFACTOR 6
#endif

#if LORA_MODE == 8
  #define LORA_EXPLICITMODE
  #define LORA_CODERATE 5
  #define LORA_BW 20.8
  #define LORA_SPREADFACTOR 7
#endif

#if LORA_MODE == 9
  #define LORA_CODERATE 5
  #define LORA_BW 62.5
  #define LORA_SPREADFACTOR 6
#endif


#endif


