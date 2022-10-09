// ADC.cpp

/***********************************************************************************
* #includes
*
* #include user defined headers, followed by 3rd party library headers, then standard 
* library headers, with the headers in each section sorted alphabetically.
************************************************************************************/
#include "ADC.h"
#include "Settings.h"


//===============================================================================
// Read an external voltage from an analog PIN by referencing the 1.1v internal reference  
// DO NOT CONNECT A HIGHER VOLTAGE THAN THE PIN IS RATED FOR (or use a voltage divider)
float readExternalVoltage()
{
  float internalV = 0.0;
  float externalV = 0.0;

  // Get a reference to compare the external voltage with
  internalV = readVCC();
  
  // Read the external voltage
  if (USE_EXTERNAL_VOLTAGE)
  {
#ifdef ATMEGA1284P  // If VCC is below 2.1V, internal voltage reference of 1.1V should not be used.
  analogReference(DEFAULT);
#else
    analogReference(DEFAULT);
#endif
    for (int i = 1; i <=15; i++)
    {
      externalV += analogRead(EXTERNALVOLTAGE_PIN);
      delay(15);
    }
    externalV = externalV / 15;
    return ((internalV / SAMPLE_RES) * externalV * DIVIDER_RATIO) + EXT_OFFSET;
  }
  else
  {
    return 0.0;
  }
}


//===============================================================================
// Read the VCC voltage by referencing the 1.1v internal reference
// Useful for checking battery voltage.
float readVCC() 
{
  long result;
  long avgmv = 0;
  const int reps = 15;

  // Measure the voltage 10 times and take average
  for (int i = 1; i <= reps; i++ )
  {
    // Read 1.1V reference against AVcc
#ifdef ATMEGA1284P
    ADMUX = (0<<REFS1) | (1<<REFS0) | (0<<ADLAR)| (1<<MUX4) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (0<<MUX0);
#else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif
    delay(15); // Wait for Vref to settle
    ADCSRA |= _BV(ADSC); // Enable the ADC
    while (bit_is_set(ADCSRA, ADSC));
    result = ADCW; // Reading register "ADCW" takes care of how to read ADCL and ADCH.
    result = 1126400L / result; // Back-calculate AVcc in mV
    avgmv+=result;
  }
  return (avgmv / reps / 1000.0) + VCC_OFFSET;
}


//===============================================================================
// Read the internal chip temperature by referencing the 1.1v internal reference voltage
float readTemp()
{
  unsigned int wADC;
  float t;

  // The internal temperature has to be used
  // with the internal reference of 1.1V.

  // Set the internal reference and mux.
  ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
  ADCSRA |= _BV(ADEN);  // enable the ADC

  delay(10);            // wait for voltages to become stable.
  ADCSRA |= _BV(ADSC);  // Start the ADC
  while (bit_is_set(ADCSRA, ADSC));   // Detect end-of-conversion
  wADC = ADCW;  // Reading register "ADCW" takes care of how to read ADCL and ADCH.
  t = (wADC - 324.31 ) / 1.22;  // The offset of 324.31 could be wrong. It is just an indication.
  return (t);  // The returned temperature is in degrees Celcius.
}
