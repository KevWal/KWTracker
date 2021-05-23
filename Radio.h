// Radio.h

#ifndef RADIO_H
#define RADIO_H

#include <SoftwareSerial.h>
#include <Arduino.h>

void SetupRTTY();

void SetupFSK();

void SetupLoRa();

void ResetRadio();

void SetupRadio();

void sendRTTY(String TxLine);

void sendLoRa(String TxLine);

extern SoftwareSerial SerialGPS;

#endif