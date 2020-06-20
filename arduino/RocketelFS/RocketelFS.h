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
#define RFS_MODE_INIT (0)
#define RFS_MODE_READ (1)
#define RFS_MODE_WRITE (2)

// data record length bytes
#define RFS_DATA_RECORD_LEN (6)

// BLE params
#define RFS_BLE_TXPOWER_READ (2)
#define RFS_BLE_TXPOWER_WRITE (8)

// BLE UUID for Telemetry Data Service (TDS) : 01690000-5df1-4174-a537-33891e600690
const uint8_t UUID128_SVC_TDS[16] = {
    0x90, 0x06, 0x60, 0x1e, 0x89, 0x33, 0x37, 0xa5,
    0x74, 0x41, 0xf1, 0x5d, 0x00, 0x00, 0x69, 0x01  };
// BLE UUID for TDS:timestamp_ms : 01690002-5df1-4174-a537-33891e600690
const uint8_t UUID128_CHR_TDS_TIMESTAMP_MS[16] = {
    0x90, 0x06, 0x60, 0x1e, 0x89, 0x33, 0x37, 0xa5,
    0x74, 0x41, 0xf1, 0x5d, 0x02, 0x00, 0x69, 0x01  };
// BLE UUID for TDS:pressure_pa : 01690003-5df1-4174-a537-33891e600690
const uint8_t UUID128_CHR_TDS_PRESSURE_PA[16] = {
    0x90, 0x06, 0x60, 0x1e, 0x89, 0x33, 0x37, 0xa5,
    0x74, 0x41, 0xf1, 0x5d, 0x03, 0x00, 0x69, 0x01  };
// BLE UUID for TDS:mode_string : 01690031-5df1-4174-a537-33891e600690
const uint8_t UUID128_CHR_TDS_MODE_STRING[16] = {
    0x90, 0x06, 0x60, 0x1e, 0x89, 0x33, 0x37, 0xa5,
    0x74, 0x41, 0xf1, 0x5d, 0x31, 0x00, 0x69, 0x01  };


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

    int getMode() {return _mode;}

    int updateBLEBatteryLevel(bool newMeasurement);
    // might want to make low level functions like this private eventually

    // expose BLE services and characteristics
    BLEDis bledis; // DIS (Device Information Service) helper object
    BLEBas blebas;  // BAS (BAttery Service) helper object

    // custom telemetry data service
    BLEService bletds;
    BLECharacteristic bletds_log_index;
    BLECharacteristic bletds_timestamp_ms;
    BLECharacteristic bletds_pressure_pa;
    BLECharacteristic bletds_altitude_m;
    BLECharacteristic bletds_max_altitude_m;
    BLECharacteristic bletds_altitude_algorithm;
    BLECharacteristic bletds_altitude_ref;
    BLECharacteristic bletds_pressure_offset_pa;
    BLECharacteristic bletds_altitude_offset_m;
    BLECharacteristic bletds_mode_string;

    // BLE callbacks
    static void bleConnectCallback(uint16_t conn_handle);
    static void bleDisconnectCallback(uint16_t conn_handle, uint8_t reason);
    // note: callbacks must be declared static

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

    // private methods
    // none yet

};

#endif
