/*
  RocketelFS.h
  Library for Model Rocket Telemetry Project using Adafruit Feather nRF5280 Sense
  Created by Neil Jacklin, June 17, 2020.
  MIT License.
*/

#include "Arduino.h"
#include "RocketelFS.h"

// globals --------------------------------------------------------------------
// some of these objects don't work as private variables :-(
Adafruit_FlashTransport_QSPI flashTransport;
Adafruit_SPIFlash flash(&flashTransport);
FatFileSystem fatfs;

Adafruit_BMP280 bmp; // I2C (with no arguments)

// Constructor ----------------------------------------------------------------
RocketelFS::RocketelFS()
{
  // nothing to do here
}

// init/begin -----------------------------------------------------------------
// returns true on success, false on failure
bool RocketelFS::begin()
{
  // PIN setup
  // set up user switch pin to digital input mode
  pinMode(PIN_USERSW,INPUT);
  // set up built-in LED for output
  pinMode(LED_BUILTIN, OUTPUT);
  // setup battery ADC PIN: none necessary

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
    Serial.println(F("ERROR: Could not find a valid BMP280 sensor, check wiring!"));
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

  // BLE Telemetry Data Service
  bletds              = BLEService(UUID128_SVC_TDS);
  bletds.begin(); // must call service.begin() before adding any characteristics

  // TDS:timestamp_ms (uint32) characteristic
  bletds_timestamp_ms = BLECharacteristic(UUID128_CHR_TDS_TIMESTAMP_MS);
  bletds_timestamp_ms.setProperties(CHR_PROPS_NOTIFY);
  bletds_timestamp_ms.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  bletds_timestamp_ms.setFixedLen(4);
  bletds_timestamp_ms.begin();

  // TDS:timestamp_ms_str (utf8s) characteristic
  bletds_timestamp_ms_str = BLECharacteristic(UUID128_CHR_TDS_TIMESTAMP_MS_STR);
  bletds_timestamp_ms_str.setProperties(CHR_PROPS_NOTIFY);
  bletds_timestamp_ms_str.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  bletds_timestamp_ms_str.setMaxLen(20); // "Time(ms):#####"
  bletds_timestamp_ms_str.begin();

  // TDS:pressure_pa (uint32) characteristic
  bletds_pressure_pa  = BLECharacteristic(UUID128_CHR_TDS_PRESSURE_PA);
  bletds_pressure_pa.setProperties(CHR_PROPS_NOTIFY);
  bletds_pressure_pa.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  bletds_pressure_pa.setFixedLen(4);
  bletds_pressure_pa.begin();

  // TDS:pressure_pa_str (utf8s) characteristic
  bletds_pressure_pa_str  = BLECharacteristic(UUID128_CHR_TDS_PRESSURE_PA_STR);
  bletds_pressure_pa_str.setProperties(CHR_PROPS_NOTIFY);
  bletds_pressure_pa_str.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  bletds_pressure_pa_str.setMaxLen(20); // "P(Pa):######"
  bletds_pressure_pa_str.begin();

  // TDS:mode_str (string) characteristic
  bletds_mode_str  = BLECharacteristic(UUID128_CHR_TDS_MODE_STR);
  bletds_mode_str.setProperties(CHR_PROPS_READ);
  bletds_mode_str.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  bletds_mode_str.setMaxLen(20); // "MODE:XXXXX"
  bletds_mode_str.begin();
  // bletds_mode_str.write("INIT"); // default value

  // BLE other services... TODO

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
  _mode = RFS_MODE_READ;
  bletds_mode_str.write("READ");

  // set initilized flag and exit
  _bInit = true;
  return _bInit;
}

// public methods -------------------------------------------------------------

// get flash (QPSI flash chip) ID
uint32_t RocketelFS::getFlashJEDECID()
{
  return flash.getJEDECID();
}

// read pressure sensor, set relevant private variables
// note this includes setting altitude derived measurement
void RocketelFS::readPressureSensor() {
  // convention: set last sensor reading time immediately before read
  _lastSensorReadingTimeMs = _lastPressureSensorReadingTimeMs = millis();
  
  // record pressure and temp measurements
  _pressurePa = bmp.readPressure();
  _tempDegC = bmp.readTemperature();

  // compute altitude
  computeAltitude();
}

// read and return battery level
int RocketelFS::readBattery() {
  Serial.print(F("DEBUG: analogRead(PIN_BATTERYADC) = "));
  Serial.println(analogRead(PIN_BATTERYADC));
  _batteryVoltage = ADC_LSB_MV * (float)analogRead(PIN_BATTERYADC) / 1000.0f;
  _batteryLevel = _batteryVoltage / RFS_BATTERY_VOLTAGE_100PCT * 100.0f + 0.5f;
  _batteryLevel = constrain(_batteryLevel,0,100);

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
  bletds_timestamp_ms_str.notify(_bleTimestampMsStr);

  // pressure characteristics
  // TODO: move to mode WRITE only?  but it's useful...
  bletds_pressure_pa.notify(&pressurePa_uint32,4);
  sprintf(_blePressurePaStr,"P(Pa):%d",pressurePa_uint32);
  bletds_pressure_pa_str.notify(_blePressurePaStr);

  // altitude characteristics
  // TODO

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

// Callback invoked when a BLE connection is made
//   conn_handle connection where this event happens
void RocketelFS::bleConnectCallback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print(F("DEBUG: BLE Connected to "));
  Serial.println(central_name);
}

// Callback invoked when a BLE connection is dropped
//   conn_handle connection where this event happens
//   reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
void RocketelFS::bleDisconnectCallback(uint16_t conn_handle, uint8_t reason)
{
  // (void) conn_handle;
  // (void) reason;

  Serial.print(F("DEBUG: BLE Disconnected, reason = 0x"));
  Serial.println(reason, HEX);
}

// private methods ------------------------------------------------------------

// compute altitude
// this will take the current _pressurePa, _altitudeAlgorithm, and related data,
// and compute the alititude measurement, and update maxAltitude if necessary.
// returns the computed altitude in m
float RocketelFS::computeAltitude() {
  // error checking
  if ( strcmp(_altitudeAlgorithm,"1A") == 0 && _altitudeOffsetM != 0.0f ) 
    Serial.println(F("WARNING: altitude offset not set right for algorithm 1A"));
  if ( strcmp(_altitudeAlgorithm,"1B") == 0 && _pressureOffsetPa != 101325.0f ) 
    Serial.println(F("WARNING: pressure offset not set right for algorithm 1B"));

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
