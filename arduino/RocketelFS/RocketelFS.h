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

// ADC pin for reading battery voltage
#define PIN_BATTERYADC PIN_A6
#define ADC_LSB_MV (0.87890625f)
#define RFS_BATTERY_VOLTAGE_100PCT (3.7f)

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

    int readBatteryLevel();
    float getBatteryVoltage() {return _batteryVoltage;}
    int getBatteryLevel() {return _batteryLevel;}

    int updateBLEBatteryLevel(bool newMeasurement);
    // might want to make low level functions like this private eventually

    // expose BLE services and characteristics
    BLEDis bledis; // DIS (Device Information Service) helper object
    BLEBas blebas;  // BAS (BAttery Service) helper object

  private:

    // initialized flag
    bool _bInit = false;

    // battery voltage and level
    float _batteryVoltage = 0.0f;
    int _batteryLevel = 0;

    // Mode
    int _mode = 0;

    // BLE name
    char _bleName[32];

    // buffers
    byte _rwBuffer[RFS_DATA_RECORD_LEN];

};

#endif
