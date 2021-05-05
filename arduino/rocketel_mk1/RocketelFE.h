/*
  RocketelFE.h
  Library for Model Rocket Telemetry Project using Adafruit Feather nRF5280 Express
  Created by Neil Jacklin, June 15, 2020.
  MIT License.
*/

#ifndef ROCKETEL_FE_H
#define ROCKETEL_FE_H

#include "Arduino.h"

#include <SPI.h>
#include <SdFat.h>
#include <Adafruit_SPIFlash.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <bluefruit.h>

// constants ------------------------------------------------------------------
// default sea level pressure
#define RFE_DEFAULTSEALEVELPRESSURE_PA (101325)

// pin for user switch
#define PIN_USERSW PIN_BUTTON1
#define RFE_BUTTON_PRESSED_VALUE (0)

// modes
#define RFE_MODE_READ (1)
#define RFE_MODE_WRITE (2)

// data record length bytes
#define RFE_DATA_RECORD_LEN (6)

// BLE params
#define RFE_BLE_TXPOWER_READ (2)
#define RFE_BLE_TXPOWER_WRITE (8)

// conversion factors
#define RFE_CONVERT_M_TO_FT (3.28084f)

// class declaration ----------------------------------------------------------
class RocketelFE
{
  public:
    // Constructor
    RocketelFE();

    // public methods
    bool begin();
    bool initialized() {return _bInit;}

    uint32_t getFlashJEDECID();

  private:
    // flash related objects
    // Adafruit_FlashTransport_QSPI _flashTransport;
    // Adafruit_SPIFlash _flash;
    // FatFileSystem _fatfs;

    // BMP388 sensor object
    Adafruit_BMP3XX _bmp;

    // initialized flag
    bool _bInit = false;

    // Mode
    int _mode = 0;

    // BLE name
    char _bleName[32];

    // buffers
    byte _rwBuffer[RFE_DATA_RECORD_LEN];

};

#endif
