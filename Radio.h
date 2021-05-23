// Radio.h

#ifndef RADIO_H
#define RADIO_H

#include "Settings.h"
#include <Arduino.h>

void SetupRTTY();

void SetupFSK();

void SetupLoRa();

void ResetRadio();

void SetupRadio();

void sendRTTY(String TxLine);

void sendLoRa(String TxLine);

//If SerialGPS is defined then we are using a Hardware serial port and dont need Software Serial
#ifndef SERIALGPS
  #include <SoftwareSerial.h>
  extern SoftwareSerial SERIALGPS;
#endif

#endif