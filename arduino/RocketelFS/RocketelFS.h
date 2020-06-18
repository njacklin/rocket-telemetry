/*
  RocketelFS.h
  Library for Model Rocket Telemetry Project using Adafruit Feather nRF5280 Sense
  Created by Neil Jacklin, June 17, 2020.
  MIT License.
*/

#ifndef ROCKETEL_FS_H
#define ROCKETEL_FS_H

#include "Arduino.h"

#include <SPI.h>
#include <SdFat.h>
#include <Adafruit_SPIFlash.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <bluefruit.h>

// constants ------------------------------------------------------------------
// default sea level pressure
#define RFS_DEFAULTSEALEVELPRESSURE_PA (101325)

// pin for user switch
#define PIN_USERSW PIN_BUTTON1
#define RFS_BUTTON_PRESSED_VALUE (0)

// modes
#define RFS_MODE_READ (1)
#define RFS_MODE_WRITE (2)

// data record length bytes
#define RFS_DATA_RECORD_LEN (6)

// BLE params
#define RFS_BLE_TXPOWER_READ (2)
#define RFS_BLE_TXPOWER_WRITE (8)

// I2C addresses
#define RFS_I2C_ADDR_BMP280 (0x77)

// conversion factors
#define RFS_CONVERT_M_TO_FT (3.28084f)

// class declaration ----------------------------------------------------------
class RocketelFS
{
  public:
    // Constructor
    RocketelFS();

    // public methods
    bool begin();
    bool initialized() {return _bInit;}

    uint32_t getFlashJEDECID();
    float readPressurePa();

  private:

    // initialized flag
    bool _bInit = false;

    // Mode
    int _mode = 0;

    // BLE name
    char _bleName[32];

    // buffers
    byte _rwBuffer[RFS_DATA_RECORD_LEN];

};

#endif
