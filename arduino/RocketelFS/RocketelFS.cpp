/*
  RocketelFS.h
  Library for Model Rocket Telemetry Project using Adafruit Feather nRF5280 Sense
  Created by Neil Jacklin, June 17, 2020.
  MIT License.
*/

#include "RocketelFS.h"

// globals --------------------------------------------------------------------
// some of these objects don't work as private variables :-(
Adafruit_FlashTransport_QSPI flashTransport;
Adafruit_SPIFlash flash(&flashTransport);
FatFileSystem fatfs;

Adafruit_BMP280 bmp; // I2C (with no arguments)

Adafruit_LSM6DS33 lsm6ds33;

// Constructor ----------------------------------------------------------------
RocketelFS::RocketelFS()
{
  // nothing to do here
  Serial.println(F("WARNING: RocketelFS intended to be used as a static class"));
}

// init/begin -----------------------------------------------------------------
// returns true on success, false on failure
bool RocketelFS::init()
{
  // init once
  if (_bInit) {
    Serial.println(F("WARNING: RocketelFS already initialized, aborting init()"));
    return false;
  }

  // PIN setup
  // set up user switch pin to digital input mode
  pinMode(PIN_USERSW,INPUT);
  // set up built-in LED for output
  pinMode(LED_BUILTIN, OUTPUT);
  // setup battery ADC 
  // following the lead of https://learn.adafruit.com/adafruit-feather-sense/nrf52-adc
  // set the analog reference to 3.0V (default = 3.6V)
  analogReference(AR_INTERNAL_3_0);
  // set the resolution to 12-bit (0..4095)
  analogReadResolution(12); // Can be 8, 10, 12 or 14
  delay(100); // delay 100 ms to let ADC settle
  // set _batteryADCvoltPerLsb = (voltage_div_comp) * (ADC voltage max / ADC output max)
  _batteryADCvoltPerLsb = 2.0f * ( 3.0f / 4096.0f );
  readBattery();

  // intitalize flash and mount FAT file system
  if (!flash.begin()) {
    Serial.println(F("ERROR: failed to initialize flash chip!"));
    return false;
  }

  if (!fatfs.begin(&flash)) {
    Serial.println(F("ERROR: failed to mount filesystem!"));
    Serial.println(F("ERROR: ensure chip formatted with the SdFat_format example"));
    return false;
  }

  // init pressure sensor
  if (!bmp.begin(RFS_I2C_ADDR_BMP280)) {
    Serial.println(F("ERROR: Could not find a valid BMP280 sensor!"));
    return false;
  }

  bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,    /* Operating Mode. Normal reads every standby time. */
    Adafruit_BMP280::SAMPLING_X1,    /* Temp. oversampling */
    Adafruit_BMP280::SAMPLING_X4,    /* Pressure oversampling */
    Adafruit_BMP280::FILTER_X2,      /* IIR Filtering. */
    Adafruit_BMP280::STANDBY_MS_63); /* Standby time.  Should be less than real sampling time. */

  // first readings are no good, so burn off a few
  bmp.readPressure(); delay(100); bmp.readPressure();

  // set altitude settings for initial altitude algorithm
  changeAltitudeAlgorithm(_altitudeAlgorithm);

  // initialize acceleration and gyromter sensor
  if (!lsm6ds33.begin_I2C()) {
    Serial.println(F("ERROR: Could not find a valid LSM6DS33 sensor!"));
    return false;
  }

  // LSM6DS_ACCEL_RANGE_2_G
  // LSM6DS_ACCEL_RANGE_4_G
  // LSM6DS_ACCEL_RANGE_8_G
  // LSM6DS_ACCEL_RANGE_16_G
  lsm6ds33.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);

  // LSM6DS_GYRO_RANGE_125_DPS
  // LSM6DS_GYRO_RANGE_250_DPS
  // LSM6DS_GYRO_RANGE_500_DPS
  // LSM6DS_GYRO_RANGE_1000_DPS
  // LSM6DS_GYRO_RANGE_2000_DPS
  lsm6ds33.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);

  // rate settings
  // LSM6DS_RATE_SHUTDOWN // Shutdown
  // LSM6DS_RATE_12_5_HZ // 12.5 Hz (80 ms period)
  // LSM6DS_RATE_26_HZ // 26 Hz (38 ms period)
  // LSM6DS_RATE_52_HZ // 52 Hz (1.9 ms period) 
  // LSM6DS_RATE_104_HZ // 104 Hz (9.6 ms period)
  // LSM6DS_RATE_208_HZ // 208 Hz (4.8 ms period)
  // LSM6DS_RATE_416_HZ // 416 Hz (2.4 ms period)
  // LSM6DS_RATE_833_HZ // 833 Hz (1.2 ms period)
  // LSM6DS_RATE_1_66K_HZ // 1.66 kHz
  // LSM6DS_RATE_3_33K_HZ // 3.33 kHz
  // LSM6DS_RATE_6_66K_HZ // 6.66 kHz

  lsm6ds33.setAccelDataRate(LSM6DS_RATE_12_5_HZ);

  lsm6ds33.setGyroDataRate(LSM6DS_RATE_12_5_HZ);

  // magic copied from adafruit_lsm6ds33_test
  lsm6ds33.configInt1(false, false, true); // accelerometer DRDY on INT1
  lsm6ds33.configInt2(false, true, false); // gyro DRDY on INT2

  // BLE init

  // init Bluefruit
  if (!Bluefruit.begin()) {
    Serial.println(F("ERROR: Could not begin() Bluefruit library."));
  }

  // set connect and disconnect callbacks
  Bluefruit.Periph.setConnectCallback(bleConnectCallback);
  Bluefruit.Periph.setDisconnectCallback(bleDisconnectCallback);

  // set Tx power and name
  Bluefruit.setTxPower(RFS_BLE_TXPOWER_READ);
  Bluefruit.setName(_bleName);
  // warning: Don't know if the above two lines need to be in that order.
  // I think each one adds something to the advertising data buffer.
  // TODO: look into docs/code to verify

  // add BLE services and characteristics

  // init BLE Device Information Service (DIS)
  // set some reasonable default values
  bledis.setManufacturer(_bleManufacturerStr);
  bledis.setModel(_bleModelStr);
  bledis.begin();

  // init BLE BAttery Service (BAS)
  blebas.begin();
  blebas.write(_batteryLevel);

  // BLE Telemetry Data Service (TDS)
  bletds              = BLEService(UUID128_SVC_TDS);
  bletds.begin(); // must call service.begin() before adding any characteristics

  // TDS:log_index (uint8) characteristic
  bletds_log_index = BLECharacteristic(UUID128_CHR_TDS_LOG_INDEX);
  bletds_log_index.setProperties(CHR_PROPS_READ);
  bletds_log_index.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  bletds_log_index.setFixedLen(1);
  bletds_log_index.begin();

  // TDS:log_index_str (utf8s) characteristic
  bletds_log_index_str = BLECharacteristic(UUID128_CHR_TDS_LOG_INDEX_STR);
  bletds_log_index_str.setProperties(CHR_PROPS_READ);
  bletds_log_index_str.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  bletds_log_index_str.setMaxLen(20); // "Log:##"
  bletds_log_index_str.begin();

  // TDS:timestamp_ms (uint32) characteristic
  bletds_timestamp_ms = BLECharacteristic(UUID128_CHR_TDS_TIMESTAMP_MS);
  bletds_timestamp_ms.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  bletds_timestamp_ms.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  bletds_timestamp_ms.setFixedLen(4);
  bletds_timestamp_ms.begin();

  // TDS:timestamp_ms_str (utf8s) characteristic
  bletds_timestamp_ms_str = BLECharacteristic(UUID128_CHR_TDS_TIMESTAMP_MS_STR);
  bletds_timestamp_ms_str.setProperties(CHR_PROPS_READ);
  bletds_timestamp_ms_str.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  bletds_timestamp_ms_str.setMaxLen(20); // "Time(ms):#####"
  bletds_timestamp_ms_str.begin();

  // TDS:pressure_pa (uint32) characteristic
  bletds_pressure_pa = BLECharacteristic(UUID128_CHR_TDS_PRESSURE_PA);
  bletds_pressure_pa.setProperties(CHR_PROPS_READ);
  bletds_pressure_pa.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  bletds_pressure_pa.setFixedLen(4);
  bletds_pressure_pa.begin();

  // TDS:pressure_pa_str (utf8s) characteristic
  bletds_pressure_pa_str = BLECharacteristic(UUID128_CHR_TDS_PRESSURE_PA_STR);
  bletds_pressure_pa_str.setProperties(CHR_PROPS_READ);
  bletds_pressure_pa_str.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  bletds_pressure_pa_str.setMaxLen(20); // "P(Pa):######"
  bletds_pressure_pa_str.begin();

  // TDS:altitude_m (float) characteristic
  bletds_altitude_m = BLECharacteristic(UUID128_CHR_TDS_ALTITUDE_M);
  bletds_altitude_m.setProperties(CHR_PROPS_READ);
  bletds_altitude_m.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  bletds_altitude_m.setFixedLen(4);
  bletds_altitude_m.begin();

  // TDS:altitude_str (utf8s) characteristic
  bletds_altitude_str = BLECharacteristic(UUID128_CHR_TDS_ALTITUDE_STR);
  bletds_altitude_str.setProperties(CHR_PROPS_READ);
  bletds_altitude_str.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  bletds_altitude_str.setMaxLen(20); // "Alt(units)):######"
  bletds_altitude_str.begin();

  // TDS:max_altitude_m (float) characteristic
  bletds_max_altitude_m = BLECharacteristic(UUID128_CHR_TDS_MAX_ALTITUDE_M);
  bletds_max_altitude_m.setProperties(CHR_PROPS_READ|CHR_PROPS_NOTIFY);
  bletds_max_altitude_m.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  bletds_max_altitude_m.setFixedLen(4);
  bletds_max_altitude_m.begin();

  // TDS:max_altitude_str (utf8s) characteristic
  bletds_max_altitude_str = BLECharacteristic(UUID128_CHR_TDS_MAX_ALTITUDE_STR);
  bletds_max_altitude_str.setProperties(CHR_PROPS_READ);
  bletds_max_altitude_str.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  bletds_max_altitude_str.setMaxLen(20); // "MaxAlt(units)):######"
  bletds_max_altitude_str.begin();

  // TDS:mode_str (string) characteristic
  bletds_mode_str = BLECharacteristic(UUID128_CHR_TDS_MODE_STR);
  bletds_mode_str.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  bletds_mode_str.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  bletds_mode_str.setMaxLen(20); // "MODE:XXXXX"
  bletds_mode_str.begin();
  // bletds_mode_str.write("MODE:INIT"); // default value

  // BLE Telemetry Config Service (TCFGS)
  bletcfgs = BLEService(UUID128_SVC_TCFGS);
  bletcfgs.begin(); // must call service.begin() before adding any characteristics

  // TCFGS:altitude_algorithm_str (utf8s) characteristic
  bletcfgs_altitude_algorithm_str = BLECharacteristic(UUID128_CHR_TCFGS_ALTITUDE_ALGORITHM_STR);
  bletcfgs_altitude_algorithm_str.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  bletcfgs_altitude_algorithm_str.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  bletcfgs_altitude_algorithm_str.setWriteCallback(bletcfgsWriteCallback);
  bletcfgs_altitude_algorithm_str.setMaxLen(3); // {'1A','1B','2A','2B'} 
  bletcfgs_altitude_algorithm_str.begin();
  bletcfgs_altitude_algorithm_str.write(_altitudeAlgorithm); // default value

  // TCFGS:altitude_ref_str (utf8s) characteristic
  bletcfgs_altitude_ref_str = BLECharacteristic(UUID128_CHR_TCFGS_ALTITUDE_REF_STR);
  bletcfgs_altitude_ref_str.setProperties(CHR_PROPS_READ);
  bletcfgs_altitude_ref_str.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  bletcfgs_altitude_ref_str.setMaxLen(4); // {'MSL','AGL'} 
  bletcfgs_altitude_ref_str.begin();
  bletcfgs_altitude_ref_str.write(_altitudeRef); // default value

  // TCFGS:pressure_offset_pa (float) characteristic
  bletcfgs_pressure_offset_pa = BLECharacteristic(UUID128_CHR_TCFGS_PRESSURE_OFFSET_PA);
  bletcfgs_pressure_offset_pa.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  bletcfgs_pressure_offset_pa.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  bletcfgs_pressure_offset_pa.setWriteCallback(bletcfgsWriteCallback);
  bletcfgs_pressure_offset_pa.setFixedLen(4);  
  bletcfgs_pressure_offset_pa.begin();
  bletcfgs_pressure_offset_pa.write32((uint32_t)_pressureOffsetPa); // default

  // TCFGS:altitude_offset_m (float) characteristic
  bletcfgs_altitude_offset_m = BLECharacteristic(UUID128_CHR_TCFGS_ALTITUDE_OFFSET_M);
  bletcfgs_altitude_offset_m.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  bletcfgs_altitude_offset_m.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  bletcfgs_altitude_offset_m.setWriteCallback(bletcfgsWriteCallback);
  bletcfgs_altitude_offset_m.setFixedLen(4);  
  bletcfgs_altitude_offset_m.begin();
  bletcfgs_pressure_offset_pa.write32((uint32_t)_altitudeOffsetM); // default 

  // TCFGS:altitude_str_units (utf8s) characteristic
  bletcfgs_altitude_str_units = BLECharacteristic(UUID128_CHR_TCFGS_ALTITUDE_STR_UNITS);
  bletcfgs_altitude_str_units.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  bletcfgs_altitude_str_units.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  bletcfgs_altitude_str_units.setWriteCallback(bletcfgsWriteCallback);
  bletcfgs_altitude_str_units.setMaxLen(3); // {'m','ft'} 
  bletcfgs_altitude_str_units.begin();
  bletcfgs_altitude_str_units.write(_altitudeStrUnits); // default value

  // TCFGS:last_log_index (uint8) characteristic
  bletcfgs_last_log_index = BLECharacteristic(UUID128_CHR_TCFGS_LAST_LOG_INDEX);
  bletcfgs_last_log_index.setProperties(CHR_PROPS_READ);
  bletcfgs_last_log_index.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  bletcfgs_last_log_index.setFixedLen(1); 
  bletcfgs_last_log_index.begin();
  if (readLastLogInfo())
    bletcfgs_last_log_index.write8((int8_t)_lastLogIndex);
  else
    Serial.println(F("ERROR: readLastLogInfo() failed inside RocketelFS::init()"));
  

  // BLE Telemetry Command Service (TCMDS)
  bletcmds = BLEService(UUID128_SVC_TCMDS);
  bletcmds.begin(); // must call service.begin() before adding any characteristics

  // TCMDS:goto_mode_read (uint8) characteristic
  bletcmds_goto_mode_read = BLECharacteristic(UUID128_CHR_TCMDS_GOTO_MODE_READ);
  bletcmds_goto_mode_read.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  bletcmds_goto_mode_read.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  bletcmds_goto_mode_read.setFixedLen(1); 
  bletcmds_goto_mode_read.begin();
  bletcmds_goto_mode_read.write8(0);

  // TCMDS:goto_mode_write (uint8) characteristic
  bletcmds_goto_mode_write = BLECharacteristic(UUID128_CHR_TCMDS_GOTO_MODE_WRITE);
  bletcmds_goto_mode_write.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  bletcmds_goto_mode_write.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  bletcmds_goto_mode_write.setFixedLen(1); 
  bletcmds_goto_mode_write.begin();
  bletcmds_goto_mode_write.write8(0);

  // TCMDS:open_log (uint8) characteristic
  bletcmds_open_log = BLECharacteristic(UUID128_CHR_TCMDS_OPEN_LOG);
  bletcmds_open_log.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  bletcmds_open_log.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  bletcmds_open_log.setFixedLen(1); 
  bletcmds_open_log.begin();
  bletcmds_open_log.write8(0);

  // TCMDS:delete_log (uint8) characteristic
  bletcmds_delete_log = BLECharacteristic(UUID128_CHR_TCMDS_DELETE_LOG);
  bletcmds_delete_log.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  bletcmds_delete_log.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  bletcmds_delete_log.setFixedLen(1); 
  bletcmds_delete_log.begin();
  bletcmds_delete_log.write8(0);

  // TCMDS:transfer_log_uart (uint8) characteristic
  bletcmds_transfer_log_uart = BLECharacteristic(UUID128_CHR_TCMDS_TRANSFER_LOG_UART);
  bletcmds_transfer_log_uart.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  bletcmds_transfer_log_uart.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  bletcmds_transfer_log_uart.setFixedLen(1); 
  bletcmds_transfer_log_uart.begin();
  bletcmds_transfer_log_uart.write8(0);

  // TCMDS:erase_all (uint8) characteristic
  bletcmds_erase_all = BLECharacteristic(UUID128_CHR_TCMDS_ERASE_ALL);
  bletcmds_erase_all.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  bletcmds_erase_all.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  bletcmds_erase_all.setFixedLen(1); 
  bletcmds_erase_all.begin();
  bletcmds_erase_all.write8(0);

  // TCMDS:log_index (uint8) characteristic
  bletcmds_log_index = BLECharacteristic(UUID128_CHR_TCMDS_LOG_INDEX);
  bletcmds_log_index.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  bletcmds_log_index.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  bletcmds_log_index.setFixedLen(1); 
  bletcmds_log_index.begin();
  bletcmds_log_index.write8(0);

  // TCMDS:ready (uint8) characteristic
  bletcmds_ready = BLECharacteristic(UUID128_CHR_TCMDS_READY);
  bletcmds_ready.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  bletcmds_ready.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  bletcmds_ready.setFixedLen(1); 
  bletcmds_ready.begin();
  bletcmds_ready.write8(1);

  // TCMDS:last_cmd_error_flag (uint8) characteristic
  bletcmds_last_cmd_error_flag = BLECharacteristic(UUID128_CHR_TCMDS_LAST_CMD_ERROR_FLAG);
  bletcmds_last_cmd_error_flag.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  bletcmds_last_cmd_error_flag.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  bletcmds_last_cmd_error_flag.setFixedLen(1); 
  bletcmds_last_cmd_error_flag.begin();
  bletcmds_last_cmd_error_flag.write8(0);

  // TCMDS:error_msg (utf8s) characteristic
  bletcmds_error_msg = BLECharacteristic(UUID128_CHR_TCMDS_ERROR_MSG);
  bletcmds_error_msg.setProperties(CHR_PROPS_READ);
  bletcmds_error_msg.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  bletcmds_error_msg.setMaxLen(20); 
  bletcmds_error_msg.begin();

  // set up BLE advertising
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  // according to Nordic docs, this flag advertises BLE only (explictly not BT classic)
  // Bluefruit.Advertising.addManufacturerData(&mfgrData,len);
  // Bluefruit.Advertising.setStopCallback(adv_stop_callback);
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in units of 0.625 ms; 32 = 20.0 ms, 244 = 152.5 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);  // Stop advertising entirely after ADV_TIMEOUT seconds, 0 = never

  // set mode to READ
  // TODO consider calling a private changeMode(toMode) method...
  _mode = RFS_MODE_READ;
  bletds_mode_str.write("MODE:READ");

  // set initilized flag and exit
  return _bInit = true;
}

// get flash (QPSI flash chip) ID
uint32_t RocketelFS::getFlashJEDECID()
{
  return flash.getJEDECID();
}

// read LASTLOGINFO.TXT
// return true if succeed (i.e. if it exists and was read successfully)
bool RocketelFS::readLastLogInfo()
{
  String lineStr;

  // try to open LASTLOG.TXT
  _file = fatfs.open("LASTLOG.TXT",FILE_READ);
  if (!_file)
    Serial.println(F("ERROR: could not open LASTLOG.TXT"));

  // read contents and store values in private variables
  lineStr = _file.readString();
  _lastLogIndex = lineStr.toInt();

  // get last number of records
  // TODO

  // if we get to the end, assume everything went okay
  return true;
}

// read pressure+temperature sensor, set relevant private variables
// note this includes setting altitude derived measurement
// returns pressure in pa as float
float RocketelFS::readPressureTempSensor() {
  // convention: set last sensor reading time immediately before read
  _lastSensorReadingTimeMs = _lastPressureTempSensorReadingTimeMs = millis();
  
  // record pressure and temp measurements
  _pressurePa = bmp.readPressure();
  _tempDegC = bmp.readTemperature();

  // compute altitude
  computeAltitude();

  return _pressurePa;
}

// read acceleration+gyrometer sensor, set relevant private variables
void RocketelFS::readAccelGyroSensor() {
  // use Adafruit sensor class to read (only interface available...)
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  lsm6ds33.getEvent(&accel, &gyro, &temp);

  // convention: set last sensor reading time immediately before read
  _lastSensorReadingTimeMs = _lastAccelGyroSensorReadingTimeMs = millis();
  
  // record acceleration measurements
  _accelXMps2 = accel.acceleration.x; // - _accelXoffsetMps2; ??
  _accelYMps2 = accel.acceleration.y; // - _accelZoffsetMps2; ??
  _accelZMps2 = accel.acceleration.z; // - _accelZoffsetMps2; ??

  // _maxAccelG = 0.0f; // TODO calculate g's and then calculate max

  // record gyrometer measurements
  _gyroXRadpS = gyro.gyro.x;
  _gyroYRadpS = gyro.gyro.y;
  _gyroZRadpS = gyro.gyro.z;
  // _gyroXOffsetRadpS = 0.0f;
  // _gyroYOffsetRadpS = 0.0f;
  // _gyroZOffsetRadpS = 0.0f;

  return;
}

// change altitude algorithm
// this will include taking "initial"/ground measurements if needed
// returns false if there is an error
// TODO: extend signature to handle 2A/2B
bool RocketelFS::changeAltitudeAlgorithm(char *algorithmStr, bool resetMaxAlt) {
  // update offsets
  if ( strcmp(algorithmStr,"1A") == 0 ) {
    _altitudeOffsetM = 0.0f;
    _pressureOffsetPa = readPressureTempSensor();
  } else if  ( strcmp(algorithmStr,"1B") == 0 ) {
    _pressureOffsetPa = 101325.0f;
    _altitudeOffsetM = 
      44330.0f * (1.0f - pow(readPressureTempSensor() / _pressureOffsetPa,0.1903));
  // } else if  ( strcmp(algorithmStr,"2A") ) { // TODO
  // } else if  ( strcmp(algorithmStr,"2B") ) { // TODO
  } else {
    return false;
  }

  // update algorithm string state variable
  strcpy(_altitudeAlgorithm,algorithmStr);

  // reset max altitude
  if ( resetMaxAlt ) 
    _maxAltitudeM  = 0.0f;
  
  // recompute altitude
  computeAltitude();

  if (debug)
    Serial.println(F("DEBUG: exiting changeAltitudeAlgorithm() successfully.")); 

  return true;
}

// read and return battery level
// based on https://learn.adafruit.com/adafruit-feather-sense/nrf52-adc
int RocketelFS::readBattery() {
  // if (debug) {
  //   Serial.print(F("DEBUG: analogRead(PIN_BATTERYADC) = "));
  //   Serial.println(analogRead(PIN_BATTERYADC));
  // }

  _batteryVoltage = _batteryADCvoltPerLsb * (float)analogRead(PIN_BATTERYADC);

  // if (debug) {
  //   Serial.print("DEBUG: battery voltage (V) = ");
  //   Serial.println(_batteryVoltage);
  // }

  if ( _batteryVoltage < 3.3f ) {
    _batteryLevel = 0;
  } else if (_batteryVoltage < 3.6f ) {
    _batteryLevel =  (_batteryVoltage - 3.3f) / 0.030f;
  } else {
    _batteryLevel =  10.0f + ((_batteryVoltage - 3.6f) * 150.0f ) + 0.5f;  
  }

  _batteryLevel = constrain(_batteryLevel,0,100);

  // if (debug) {
  //   Serial.print("DEBUG: battery level (%) = ");
  //   Serial.println(_batteryLevel);
  // }

  return _batteryLevel;
}

// update BLE battery level advertised
// return battery level
// BUG: this always returns around 0.5V == 14% right now
int RocketelFS::updateBLEBatteryLevel(bool newMeasurement = true) {
  if (newMeasurement)
    blebas.write(readBattery());
  else
    blebas.write(_batteryLevel);

  return _batteryLevel;
}

// update all the characteristics of the BLE Telemetry Data Service
// the behavior of the method is affected by _mode 
void RocketelFS::updateBLETDS() {
  uint32_t timestampMs_uint32 = (uint32_t) _lastSensorReadingTimeMs;
  // TODO: think very carefully about what timestamp to report...
  uint32_t pressurePa_uint32 = (uint32_t) _pressurePa;

  // timestamp characteristics
  // TODO: move to mode WRITE only?  but it's useful...
  bletds_timestamp_ms.notify(&timestampMs_uint32,4);
  sprintf(_bleTimestampMsStr,"Time(ms):%d",timestampMs_uint32);
  bletds_timestamp_ms_str.write(_bleTimestampMsStr);

  // pressure characteristics
  // TODO: move to mode WRITE only?  but it's useful...
  bletds_pressure_pa.notify(&pressurePa_uint32,4);
  sprintf(_blePressurePaStr,"P(Pa):%d",pressurePa_uint32);
  bletds_pressure_pa_str.notify(_blePressurePaStr);

  // altitude and max altitude characteristics
  bletds_altitude_m.notify(&_altitudeM,4);

  if ( strcmp(_altitudeStrUnits,"m") == 0 )
    sprintf(_bleAltitudeStr,"Alt(m):%.1f",_altitudeM);
  else if ( strcmp(_altitudeStrUnits,"ft") == 0 )
    sprintf(_bleAltitudeStr,"Alt(ft):%.1f",_altitudeM*RFS_CONVERT_M_TO_FT);
                                
  bletds_altitude_str.notify(_bleAltitudeStr);
  
  bletds_max_altitude_m.notify(&_maxAltitudeM,4);
  
  if ( strcmp(_altitudeStrUnits,"m") == 0 )
    sprintf(_bleMaxAltitudeStr,"MaxAlt(m):%.1f",_maxAltitudeM);
  else if ( strcmp(_altitudeStrUnits,"ft") == 0 )
    sprintf(_bleMaxAltitudeStr,"MaxAlt(ft):%.1f",_maxAltitudeM*RFS_CONVERT_M_TO_FT);
  
  bletds_max_altitude_str.notify(_bleMaxAltitudeStr);

  // mode-specific updates
  switch (_mode) {
    case RFS_MODE_READ:
      // update mode characteristic
      bletds_mode_str.write("MODE:READ");

      // TODO

      break;

    case RFS_MODE_WRITE:
      // update mode characteristic
      bletds_mode_str.write("MODE:WRITE");

      // TODO

      break;

    default:
      Serial.print(F("ERROR: Invalid mode in updateBLETDS(): "));
      Serial.println(_mode);
  }
}

// handle writes to BLE TCFGS characteristics
// only accepts writes when mode == READ
void RocketelFS::bletcfgsWriteCallback(uint16_t conn_hdl, BLECharacteristic* bchr, uint8_t* data, uint16_t len)
{
  float fval;

  if ( _mode != RFS_MODE_READ )
    return;

  if ( bchr == &bletcfgs_altitude_algorithm_str ) {
    strncpy(_altitudeAlgorithm, (char*) data, (len < 3) ? len : 3 );
    if (debug) {
      Serial.print(F("DEBUG: bletcfgsWriteCallback altitude algorithm updated to "));
      Serial.println(_altitudeAlgorithm);
    }
  } else if ( bchr == &bletcfgs_pressure_offset_pa ) {
    memcpy(&fval, data, (len < 4) ? len : 4 );
    _pressureOffsetPa = fval;
    if (debug) {
      Serial.print(F("DEBUG: bletcfgsWriteCallback pressure offset pa updated to "));
      Serial.println(_pressureOffsetPa);
    }
  } else if ( bchr == &bletcfgs_altitude_offset_m ) {
    memcpy(&fval, data, (len < 4) ? len : 4 );
    _altitudeOffsetM = fval;
    if (debug) {
      Serial.print(F("DEBUG: bletcfgsWriteCallback altitude offset pa updated to "));
      Serial.println(_altitudeOffsetM);
    }
  } else if ( bchr == &bletcfgs_altitude_str_units ) {
    strncpy(_altitudeStrUnits, (char*) data, (len < 2) ? len : 2 );
    if (debug) {
      Serial.print(F("DEBUG: bletcfgsWriteCallback altitude str units updated to "));
      Serial.println(_altitudeStrUnits);
    }
  }   

}

// check TCMDS:goto_mode_read (DEBUG)
bool RocketelFS::readBLECmdGotoRead()
{
  return bletcmds_goto_mode_read.read8();
}

// check TCMDS:goto_mode_read (DEBUG)
bool RocketelFS::readBLECmdGotoWrite()
{
  return bletcmds_goto_mode_write.read8();
}

// Callback invoked when a BLE connection is made
//   conn_handle connection where this event happens
void RocketelFS::bleConnectCallback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  if (debug) {
    Serial.print(F("DEBUG: BLE Connected to "));
    Serial.println(central_name);
  }
}

// Callback invoked when a BLE connection is dropped
//   conn_handle connection where this event happens
//   reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
void RocketelFS::bleDisconnectCallback(uint16_t conn_handle, uint8_t reason)
{
  // (void) conn_handle;
  // (void) reason;
  if (debug) {
    Serial.print(F("DEBUG: BLE Disconnected, reason = 0x"));
    Serial.println(reason, HEX);
  }
}


// Create/open new log file.  
// This function will first create a LASTLOGINDEX.TXT, then LOGNN.META, 
// and then open a LOGNN.DAT for writing
bool RocketelFS::openNewLog() 
{
  char ones, tens;

  // find first available log index

  // try to open files LOGNN.DAT, increasing 
  //   NN from 00 to 99.  if all files are full, write to LOG99.DAT.
  strcpy(_filename,"LOGNN.DAT");
  for ( tens = '0'; tens <= '9'; tens++) 
  {
    _filename[3] = tens;
    for ( ones = '0'; ones <= '9'; ones++ ) 
    {
      _filename[4] = ones;
      
      if ( !fatfs.exists(_filename) )
        goto BREAKOUTFORFOR;
    }
  }
  BREAKOUTFORFOR:

  // filename now contains the first available log file name, "LOGNN.DAT"

  // set _currentLogIndex for future reference
  _currentLogIndex = 10*(tens - '0') + (ones - '0'); 

  // write LASTLOGINDEX.TXT with current log index
  fatfs.remove("LASTLOG.TXT"); // no overwrite mode, so delete if it exists
  _file = fatfs.open("LASTLOG.TXT",FILE_WRITE); // WRITE/APPEND
  if (!_file)
    Serial.println("ERROR: could not open LASTLOG.TXT");
  
  // write log index as chars
  _file.write(tens); 
  _file.write(ones); 
  _file.println();

  // close "LASTLOGINDEX.TXT"
  _file.close();

  // write LOGNN.META
  strcpy(&_filename[6],"META"); // change _filename to "LOGNN.META"
  fatfs.remove(_filename); // no overwrite mode, so delete if it exists
  _file = fatfs.open(_filename,FILE_WRITE); // WRITE/APPEND
  if (!_file) {
    Serial.print("ERROR: could not open file: ");
    Serial.println(_filename);
  }
  
  // write log index (redundant, but good integrity checking)
  _file.print("INDEX:"); 
  _file.write(tens); 
  _file.write(ones); 
  _file.println();

  // write record format
  _file.print("FORMAT:"); _file.println(RFS_RECORD_FORMAT);

  // write altitude reference
  _file.print("ALTREF:"); _file.println(_altitudeRef);

  // write altitude algorithm
  _file.print("ALTALGO:"); _file.println(_altitudeAlgorithm);
  // write pressure offset
  _file.print("P0:"); _file.println(_pressureOffsetPa);
  // write altitude offset
  _file.print("A0:"); _file.println(_altitudeOffsetM);

  // close "LOGNN.META"
  _file.close();

  // zero out num records written counter
  _numRecordsWritten = 0;

  // open LOGNN.DAT
  strcpy(&_filename[6],"DAT"); // change _filename to "LOGNN.DAT"
  return _file.open(_filename,FILE_WRITE);

}

// write record to flash file
// returns true on success and false on failure
bool RocketelFS::writeFlashRecord() 
{
  // for reference
  // format 1 = ( [uint16] timestamp_ms, [int16] pressure_pa - 100000, [int16] altitude_dm )

  uint16_t timestampmsData;
  int16_t pressureData;
  int16_t altitudedmData;

  if ( RFS_RECORD_FORMAT == 1 ) {
      
    // prepare data values for writing
    timestampmsData = _lastSensorReadingTimeMs;
    pressureData = _pressurePa - 100000.0f + 0.5f; // add 0.5 so that truncation does rounding
    altitudedmData = _altitudeM * 10.0f + 0.5f; // add 0.5 so that truncation does rounding

    // fill up record buffer
    _recordBuffer[0] = timestampmsData;
    _recordBuffer[1] = timestampmsData >> 8;
    _recordBuffer[2] = pressureData;
    _recordBuffer[3] = pressureData >> 8;
    _recordBuffer[4] = altitudedmData;
    _recordBuffer[5] = altitudedmData >> 8;
  }
  else {
    Serial.print(F("ERROR: in RocketelFS::writeFlashRecord(): unsupported RFS_RECORD_FORMAT = "));
    Serial.print(RFS_RECORD_FORMAT);
    Serial.println();
    return false;
  }

  // write record
  for (int i = 0; i < RFS_RECORD_BUFFER_BYTES; i++ )
    _file.write(_recordBuffer[i]);     

  // increment num record counter
  _numRecordsWritten++; 

  // exit 
  return true;  

}

// flush flash writes
// Files need to be closed to committed to flash (otherwise they are just in memory).
// Also, write number of records to LASTLOGINFO.TXT
// Returns true on success, false on failure.
bool RocketelFS::flushFlashWrites()
{
  // close .DAT file
  _file.close();

  _lastFlashFlushms = millis();

  // append num records to .META file
  strcpy(&_filename[6],"META"); // change _filename to "LOGNN.META"
  _file = fatfs.open(_filename,FILE_WRITE); // WRITE/APPEND
  // if (!_file) { // need to be fast, let's skip this for now
  //   Serial.print("ERROR: could not open file: ");
  //   Serial.println(_filename);
  // }
  
  // write log index (redundant, but good integrity checking)
  _file.print("NR:"); 
  _file.print(_numRecordsWritten); 
  _file.println();

  // close .META file
  _file.close();

  // reopen .DAT file
  strcpy(&_filename[6],"DAT"); // change _filename to "LOGNN.DAT"
  _file = fatfs.open(_filename, FILE_WRITE); // WRITE/APPEND 
       
}

// open LOGNN.DAT for reading
// return true on succes, false on failure
bool RocketelFS::openLogForRead(int logIndex)
{

  // form filename
  strcpy(_filename,"LOGNN.DAT");
  _filename[3] = '0' + (logIndex / 10);
  _filename[4] = '0' + (logIndex % 10);

  if (debug) {
    Serial.print(F("DEBUG: Opening file for reading: "));
    Serial.println(_filename);
  }

  // open file
  if ( fatfs.open(_filename, FILE_READ) ) {
    _currentLogIndex = logIndex;
    return true;
  } else {
    _currentLogIndex = -1;
    return false;
  }

}

// close flash file
// return true on success, false on failure
bool RocketelFS::closeFlashFile() 
{
  return _file.close();
}

// private methods ------------------------------------------------------------

// compute altitude
// this will take the current _pressurePa, _altitudeAlgorithm, and related data,
// and compute the alititude measurement, and update maxAltitude if necessary.
// returns the computed altitude in m
float RocketelFS::computeAltitude() 
{
  // error checking
  if ( strcmp(_altitudeAlgorithm,"1A") == 0 && _altitudeOffsetM != 0.0f ) {
    Serial.println(F("WARNING: altitude offset not set right for algorithm 1A"));
    if (debug) {
      Serial.print(F("DEBUG: _altitudeOffsetM = ")); 
      Serial.println(_altitudeOffsetM);
    }
  }
  
  if ( strcmp(_altitudeAlgorithm,"1B") == 0 && _pressureOffsetPa != 101325.0f ) { 
    Serial.println(F("WARNING: pressure offset not set right for algorithm 1B"));
  }

  // compute 
  // note: calculation is the same for all algorithms, just the offsets change
  _altitudeM = 44330.0f * (1.0f - pow(_pressurePa / _pressureOffsetPa, 0.1903f)) 
               - _altitudeOffsetM;

  // update max altitude if necessary
  if (_altitudeM > _maxAltitudeM)
    _maxAltitudeM = _altitudeM;
  
  // return altitude
  return _altitudeM;
}

// debug/test functions

float RocketelFS::setMaxAltitudeM(float valueM) 
{ 
  if (!debug)
    Serial.println(F("WARNING: setMaxAltitudeM() should only be called when debug enabled."));
  
  return _maxAltitudeM = valueM; 
}


