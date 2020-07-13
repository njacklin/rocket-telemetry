/*
  RocketelFS.h
  Library for Model Rocket Telemetry Project using Adafruit Feather nRF5280 Sense
  Created by Neil Jacklin, June 17, 2020.
  MIT License.

  Set RFS_DEBUG variable to enable debug messages.
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

// ADC pin for reading battery voltage
#define PIN_BATTERYADC PIN_A6

// pin for user switch
#define PIN_USERSW PIN_BUTTON1
#define RFS_BUTTON_PRESSED_VALUE (0)

// modes
#define RFS_MODE_INIT (0)
#define RFS_MODE_READ (1)
#define RFS_MODE_WRITE (2)

// data record params
#define RFS_RECORD_FORMAT (1) 
// format 1 = ( [uint16] timestamp_ms, [int16] pressure_pa - 100000, [int16] altitude_dm )
#define RFS_RECORD_FORMAT1_BYTES (6) 

#define RFS_RECORD_BUFFER_BYTES (6) // this should be the max of avaiable formats

// BLE params
#define RFS_BLE_TXPOWER_READ (2)
#define RFS_BLE_TXPOWER_WRITE (8)

// BLE UUID for Telemetry Data Service (TDS)  : 01690000-5df1-4174-a537-33891e600690
const uint8_t UUID128_SVC_TDS[16] = {
    0x90, 0x06, 0x60, 0x1e, 0x89, 0x33, 0x37, 0xa5,
    0x74, 0x41, 0xf1, 0x5d, 0x00, 0x00, 0x69, 0x01  };
// BLE UUID for TDS:timestamp_ms              : 01690002-5df1-4174-a537-33891e600690
const uint8_t UUID128_CHR_TDS_TIMESTAMP_MS[16] = {
    0x90, 0x06, 0x60, 0x1e, 0x89, 0x33, 0x37, 0xa5,
    0x74, 0x41, 0xf1, 0x5d, 0x02, 0x00, 0x69, 0x01  };
// BLE UUID for TDS:pressure_pa               : 01690003-5df1-4174-a537-33891e600690
const uint8_t UUID128_CHR_TDS_PRESSURE_PA[16] = {
    0x90, 0x06, 0x60, 0x1e, 0x89, 0x33, 0x37, 0xa5,
    0x74, 0x41, 0xf1, 0x5d, 0x03, 0x00, 0x69, 0x01  };
// BLE UUID for TDS:altitude_m                : 01690004-5df1-4174-a537-33891e600690
const uint8_t UUID128_CHR_TDS_ALTITUDE_M[16] = {
    0x90, 0x06, 0x60, 0x1e, 0x89, 0x33, 0x37, 0xa5,
    0x74, 0x41, 0xf1, 0x5d, 0x04, 0x00, 0x69, 0x01  };
// BLE UUID for TDS:max_altitude_m            : 01690F04-5df1-4174-a537-33891e600690
const uint8_t UUID128_CHR_TDS_MAX_ALTITUDE_M[16] = {
    0x90, 0x06, 0x60, 0x1e, 0x89, 0x33, 0x37, 0xa5,
    0x74, 0x41, 0xf1, 0x5d, 0x04, 0x0F, 0x69, 0x01  };
// BLE UUID for TDS:timestamp_ms_str          : 01691002-5df1-4174-a537-33891e600690
const uint8_t UUID128_CHR_TDS_TIMESTAMP_MS_STR[16] = {
    0x90, 0x06, 0x60, 0x1e, 0x89, 0x33, 0x37, 0xa5,
    0x74, 0x41, 0xf1, 0x5d, 0x02, 0x10, 0x69, 0x01  };
// BLE UUID for TDS:pressure_pa_str           : 01691003-5df1-4174-a537-33891e600690
const uint8_t UUID128_CHR_TDS_PRESSURE_PA_STR[16] = {
    0x90, 0x06, 0x60, 0x1e, 0x89, 0x33, 0x37, 0xa5,
    0x74, 0x41, 0xf1, 0x5d, 0x03, 0x10, 0x69, 0x01  };
// BLE UUID for TDS:altitude_str              : 01691004-5df1-4174-a537-33891e600690
const uint8_t UUID128_CHR_TDS_ALTITUDE_STR[16] = {
    0x90, 0x06, 0x60, 0x1e, 0x89, 0x33, 0x37, 0xa5,
    0x74, 0x41, 0xf1, 0x5d, 0x04, 0x10, 0x69, 0x01  };
// BLE UUID for TDS:max_altitude_str          : 01691F04-5df1-4174-a537-33891e600690
const uint8_t UUID128_CHR_TDS_MAX_ALTITUDE_STR[16] = {
    0x90, 0x06, 0x60, 0x1e, 0x89, 0x33, 0x37, 0xa5,
    0x74, 0x41, 0xf1, 0x5d, 0x04, 0x1F, 0x69, 0x01  };
// BLE UUID for TDS:mode_str                  : 01690031-5df1-4174-a537-33891e600690
const uint8_t UUID128_CHR_TDS_MODE_STR[16] = {
    0x90, 0x06, 0x60, 0x1e, 0x89, 0x33, 0x37, 0xa5,
    0x74, 0x41, 0xf1, 0x5d, 0x31, 0x00, 0x69, 0x01  };

// BLE UUID for Telemetry Config Svc (TCFGS)  : 03140000-a8f4-4942-8bd6-5b9beb9db908
const uint8_t UUID128_SVC_TCFGS[16] = {
    0x08, 0xb9, 0x9d, 0xeb, 0x9b, 0x5b, 0xd6, 0x8b,
    0x42, 0x49, 0xf4, 0xa8, 0x00, 0x00, 0x14, 0x03  };
// BLE UUID for TCFGS:altitude_algorithm_str  : 031400002-a8f4-4942-8bd6-5b9beb9db908
const uint8_t UUID128_CHR_TCFGS_ALTITUDE_ALGORITHM_STR[16] = {
    0x08, 0xb9, 0x9d, 0xeb, 0x9b, 0x5b, 0xd6, 0x8b,
    0x42, 0x49, 0xf4, 0xa8, 0x02, 0x00, 0x14, 0x03  };
// BLE UUID for TCFGS:altitude_ref_str        : 031400003-a8f4-4942-8bd6-5b9beb9db908
const uint8_t UUID128_CHR_TCFGS_ALTITUDE_REF_STR[16] = {
    0x08, 0xb9, 0x9d, 0xeb, 0x9b, 0x5b, 0xd6, 0x8b,
    0x42, 0x49, 0xf4, 0xa8, 0x03, 0x00, 0x14, 0x03  };
// BLE UUID for TCFGS:pressure_offset_pa      : 031400011-a8f4-4942-8bd6-5b9beb9db908
const uint8_t UUID128_CHR_TCFGS_PRESSURE_OFFSET_PA[16] = {
    0x08, 0xb9, 0x9d, 0xeb, 0x9b, 0x5b, 0xd6, 0x8b,
    0x42, 0x49, 0xf4, 0xa8, 0x11, 0x00, 0x14, 0x03  };
// BLE UUID for TCFGS:altitude_offset_m       : 031400012-a8f4-4942-8bd6-5b9beb9db908
const uint8_t UUID128_CHR_TCFGS_ALTITUDE_OFFSET_M[16] = {
    0x08, 0xb9, 0x9d, 0xeb, 0x9b, 0x5b, 0xd6, 0x8b,
    0x42, 0x49, 0xf4, 0xa8, 0x12, 0x00, 0x14, 0x03  };
// BLE UUID for TCFGS:altitude_str_units      : 03140F001-a8f4-4942-8bd6-5b9beb9db908
const uint8_t UUID128_CHR_TCFGS_ALTITUDE_STR_UNITS[16] = {
    0x08, 0xb9, 0x9d, 0xeb, 0x9b, 0x5b, 0xd6, 0x8b,
    0x42, 0x49, 0xf4, 0xa8, 0x01, 0xF0, 0x14, 0x03  };
// BLE UUID for TCFGS:last_log_index          : 031400021-a8f4-4942-8bd6-5b9beb9db908
const uint8_t UUID128_CHR_TCFGS_LAST_LOG_INDEX[16] = {
    0x08, 0xb9, 0x9d, 0xeb, 0x9b, 0x5b, 0xd6, 0x8b,
    0x42, 0x49, 0xf4, 0xa8, 0x21, 0x00, 0x14, 0x03  };

// BLE UUD for Telemetry Command Svc (TCMDS)  : e0970000-49bc-4d61-8229-6585dfec6eb7
const uint8_t UUID128_SVC_TCMDS[16] = {
    0xb7, 0x6e, 0xec, 0xdf, 0x85, 0x65, 0x29, 0x82,
    0x61, 0x4d, 0xbc, 0x49, 0x00, 0x00, 0x97, 0xe0  };
// BLE UUID for TCMDS:goto_mode_read          : e0970001-49bc-4d61-8229-6585dfec6eb7
const uint8_t UUID128_CHR_TCMDS_GOTO_MODE_READ[16] = {
    0xb7, 0x6e, 0xec, 0xdf, 0x85, 0x65, 0x29, 0x82,
    0x61, 0x4d, 0xbc, 0x49, 0x01, 0x00, 0x97, 0xe0  };
// BLE UUID for TCMDS:goto_mode_write         : e0970002-49bc-4d61-8229-6585dfec6eb7
const uint8_t UUID128_CHR_TCMDS_GOTO_MODE_WRITE[16] = {
    0xb7, 0x6e, 0xec, 0xdf, 0x85, 0x65, 0x29, 0x82,
    0x61, 0x4d, 0xbc, 0x49, 0x02, 0x00, 0x97, 0xe0  };
// BLE UUID for TCMDS:open_log                : e0970011-49bc-4d61-8229-6585dfec6eb7
const uint8_t UUID128_CHR_TCMDS_OPEN_LOG[16] = {
    0xb7, 0x6e, 0xec, 0xdf, 0x85, 0x65, 0x29, 0x82,
    0x61, 0x4d, 0xbc, 0x49, 0x11, 0x00, 0x97, 0xe0  };
// BLE UUID for TCMDS:delete_log              : e0970012-49bc-4d61-8229-6585dfec6eb7
const uint8_t UUID128_CHR_TCMDS_DELETE_LOG[16] = {
    0xb7, 0x6e, 0xec, 0xdf, 0x85, 0x65, 0x29, 0x82,
    0x61, 0x4d, 0xbc, 0x49, 0x12, 0x00, 0x97, 0xe0  };
// BLE UUID for TCMDS:transfer_log_uart       : e0970021-49bc-4d61-8229-6585dfec6eb7
const uint8_t UUID128_CHR_TCMDS_TRANSFER_LOG_UART[16] = {
    0xb7, 0x6e, 0xec, 0xdf, 0x85, 0x65, 0x29, 0x82,
    0x61, 0x4d, 0xbc, 0x49, 0x21, 0x00, 0x97, 0xe0  };
// BLE UUID for TCMDS:erase_all               : e0970031-49bc-4d61-8229-6585dfec6eb7
const uint8_t UUID128_CHR_TCMDS_ERASE_ALL[16] = {
    0xb7, 0x6e, 0xec, 0xdf, 0x85, 0x65, 0x29, 0x82,
    0x61, 0x4d, 0xbc, 0x49, 0x31, 0x00, 0x97, 0xe0  };
// BLE UUID for TCMDS:read_log_index          : e0970101-49bc-4d61-8229-6585dfec6eb7
const uint8_t UUID128_CHR_TCMDS_LOG_INDEX[16] = {
    0xb7, 0x6e, 0xec, 0xdf, 0x85, 0x65, 0x29, 0x82,
    0x61, 0x4d, 0xbc, 0x49, 0x01, 0x01, 0x97, 0xe0  };
// BLE UUID for TCMDS:ready                   : e0970a01-49bc-4d61-8229-6585dfec6eb7
const uint8_t UUID128_CHR_TCMDS_READY[16] = {
    0xb7, 0x6e, 0xec, 0xdf, 0x85, 0x65, 0x29, 0x82,
    0x61, 0x4d, 0xbc, 0x49, 0x01, 0x0a, 0x97, 0xe0  };
// BLE UUID for TCMDS:last_cmd_error_flag     : e0970e01-49bc-4d61-8229-6585dfec6eb7
const uint8_t UUID128_CHR_TCMDS_LAST_CMD_ERROR_FLAG[16] = {
    0xb7, 0x6e, 0xec, 0xdf, 0x85, 0x65, 0x29, 0x82,
    0x61, 0x4d, 0xbc, 0x49, 0x01, 0x0e, 0x97, 0xe0  };
// BLE UUID for TCMDS:error_msg               : e0970e02-49bc-4d61-8229-6585dfec6eb7
const uint8_t UUID128_CHR_TCMDS_ERROR_MSG[16] = {
    0xb7, 0x6e, 0xec, 0xdf, 0x85, 0x65, 0x29, 0x82,
    0x61, 0x4d, 0xbc, 0x49, 0x02, 0x0e, 0x97, 0xe0  };

// TODO rewrite UUIDs to save memory

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

    // flash related methods
    uint32_t getFlashJEDECID();
    bool readLastLogInfo();

    bool openNewLog();
    bool writeFlashRecord();
    bool flushFlashWrites();

    bool openLogForRead(int logIndex);
    bool closeFlashFile();

    int getCurrentLogIndex() {return _currentLogIndex;}

    // battery related methods
    int readBattery();
    float getLastBatteryVoltage() {return _batteryVoltage;}
    int getLastBatteryLevel() {return _batteryLevel;}
    
    // pressure+temperature sensor methods
    float readPressureTempSensor();
    float getLastPresurePa() {return _pressurePa;}
    float getLastAltitudeM() {return _altitudeM;}
    float getMaxAltitudeM() {return _maxAltitudeM;}

    bool changeAltitudeAlgorithm(char *algorithmStr, bool resetMaxAlt = true);

    // mode functions
    int getMode() {return _mode;}

    // expose BLE services and characteristics
    BLEDis bledis; // DIS (Device Information Service) helper object
    BLEBas blebas;  // BAS (BAttery Service) helper object

    // custom telemetry data service
    BLEService bletds;
    BLECharacteristic bletds_log_index_str;
    BLECharacteristic bletds_timestamp_ms;
    BLECharacteristic bletds_pressure_pa;
    BLECharacteristic bletds_altitude_m;
    BLECharacteristic bletds_max_altitude_m;
    BLECharacteristic bletds_timestamp_ms_str;
    BLECharacteristic bletds_pressure_pa_str;
    BLECharacteristic bletds_altitude_str;
    BLECharacteristic bletds_max_altitude_str;
    BLECharacteristic bletds_altitude_algorithm;
    BLECharacteristic bletds_altitude_ref;
    BLECharacteristic bletds_pressure_offset_pa;
    BLECharacteristic bletds_altitude_offset_m;
    BLECharacteristic bletds_mode_str;

    // custom telemetry config service
    BLEService bletcfgs;
    BLECharacteristic bletcfgs_altitude_algorithm_str;
    BLECharacteristic bletcfgs_altitude_ref_str;
    BLECharacteristic bletcfgs_pressure_offset_pa;
    BLECharacteristic bletcfgs_altitude_offset_m;
    BLECharacteristic bletcfgs_altitude_str_units;
    BLECharacteristic bletcfgs_last_log_index;

    // custom telemetry command service
    BLEService bletcmds;
    BLECharacteristic bletcmds_goto_mode_read;
    BLECharacteristic bletcmds_goto_mode_write;
    BLECharacteristic bletcmds_open_log;
    BLECharacteristic bletcmds_delete_log;
    BLECharacteristic bletcmds_transfer_log_uart;
    BLECharacteristic bletcmds_erase_all;
    BLECharacteristic bletcmds_log_index;
    BLECharacteristic bletcmds_ready;
    BLECharacteristic bletcmds_last_cmd_error_flag;
    BLECharacteristic bletcmds_error_msg;

    // BLE callbacks
    static void bleConnectCallback(uint16_t conn_handle);
    static void bleDisconnectCallback(uint16_t conn_handle, uint8_t reason);
    // note: callbacks must be declared static

    // BLE methods
    void updateBLETDS();
    void updateBLETCFGS(); // TODO 
    void updateBLETCMDS(); // TODO

    bool readBLECmdGotoRead();
    bool readBLECmdGotoWrite();
    

    int updateBLEBatteryLevel(bool newMeasurement);
    // might want to make low level functions like this private eventually

    // misc
    float convertDegCtoF(float degC) { return degC * 1.8f + 32.0f; }

    // debug/test functions
    #ifdef RFS_DEBUG
    float setMaxAltitudeM(float valueM) { return _maxAltitudeM = valueM; }
    #endif

  private:

    // initialized flag
    bool _bInit = false;

    // Mode
    int _mode = 0;

    // Flash fields
    File _file; // can only have one open, so might as well reuse
    char _filename[64] = ""; 
    long _lastFlashFlushms;
    uint32_t _numRecordsWritten = 0;

    int16_t _lastLogIndex = -1;
    int16_t _currentLogIndex = -1;
    int16_t _recordFormat = RFS_RECORD_FORMAT;
    float _lastMaxAltitudeM = 0.0f;
    char _lastAltitudeRef[4] = "---";
    char _lastAltitudeAlgorithm[3] = "--";
    float _lastPressureOffsetPa = 0.0f;
    float _lastAltitudeOffsetM = 0.0f;
    int32_t _lastNumRecords = 0;

    // BLE 
    char _bleName[32] = "RocketelFS-1"; // TODO: verify max length
    char _bleManufacturerStr[32] = "Neil Jacklin | njtronics.com"; // TODO: verify max length
    char _bleModelStr[32] = "Mark 2"; // TODO: verify max length
    char _bleModeStr[20] = "INIT";
    char _bleLogIndexStr[20] = "";
    char _bleTimestampMsStr[20] = "";
    char _blePressurePaStr[20] = "";
    char _bleAltitudeStr[20] = "";
    char _bleMaxAltitudeStr[20] = "";

    // battery voltage and level
    float _batteryVoltage = 0.0f;
    int _batteryLevel = 0;  // 0 - 100
    float _batteryADCvoltPerLsb = 0.0f; // assigned in begin()

    // pressure and temperature data, calculated altitude, and altitude settings
    float _pressurePa = 0.0f;
    float _tempDegC = 0.0f;
    float _pressureOffsetPa = 101325.0f;
    float _altitudeM = 0.0f;
    float _maxAltitudeM = 0.0f;
    float _altitudeOffsetM = 0.0f;
    char _altitudeRef[4] = "AGL";
    char _altitudeAlgorithm[3] = "1A";
    char _altitudeStrUnits[3] = "ft";

    unsigned long _lastPressureTempSensorReadingTimeMs = 0L; 

    // accelerometer, magnetometer data
    // TODO: add

    // last sensor reading; should be updated whenever _any_ sensor is read
    unsigned long _lastSensorReadingTimeMs = 0L;

    // buffers
    byte _recordBuffer[RFS_RECORD_BUFFER_BYTES];

    // private methods

    // pressure and temp sensor
    float computeAltitude();

};

#endif
