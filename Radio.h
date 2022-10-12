// Radio.h

#ifndef RADIO_H
#define RADIO_H

#include "Settings.h"
#include <Arduino.h>

void setupRTTY();

void setupFSK();

void setupLoRa();

void resetRadio();

void setupRadio();

void sendRTTY(const char* TxLine);

void sendLoRa(const char* TxLine);

int8_t getRadioTemp();

//If SerialGPS is defined then we are using a Hardware serial port and dont need Software Serial
#ifndef SERIALGPS
  #include <SoftwareSerial.h>
  extern SoftwareSerial SERIALGPS;
#endif

#endif