/*
  RocketelFE.cpp
  Library for Model Rocket Telemetry Project using Adafruit Feather nRF5280 Express
  Created by Neil Jacklin, June 15, 2020.
  MIT License.
*/

#include "Arduino.h"
#include "RocketelFE.h"

// Constructor ----------------------------------------------------------------
RocketelFE::RocketelFE()
{
  // create flash and fatfs objects
  Adafruit_FlashTransport_QSPI _flashTransport;
  Adafruit_SPIFlash _flash(&_flashTransport);
  FatFileSystem _fatfs;

  // create BMP sensor object
  Adafruit_BMP3XX _bmp; // I2C (with no arguments)


}

// init/begin -----------------------------------------------------------------
void RocketelFE::begin()
{
  // do lots of stuff


  // set initilized flag
  _bInit = true;
}

// public methods -------------------------------------------------------------
bool RocketelFE::initialized()
{
  return _bInit;
}
