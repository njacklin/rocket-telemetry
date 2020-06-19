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

  // set name
  strcpy(_bleName,"RocketelFS-1");

  // init Bluefruit
  if (!Bluefruit.begin()) {
    Serial.println(F("ERROR: Could not begin() Bluefruit library."));
  }

  Bluefruit.setTxPower(RFS_BLE_TXPOWER_READ);
  Bluefruit.setName(_bleName);
  // warning: Don't know if the above two lines need to be in that order.
  // I think each one adds something to the advertising data buffer.
  // TODO: look into docs/code to verify

  // TODO add BLE services and characteristics

  // init BLE Device Information Service (DIS)
  // set some reasonable default values
  bledis.setManufacturer("Neil Jacklin | njtronics.com");
  bledis.setModel("Mark 2");
  bledis.begin();

  // init BLE BAttery Service (BAS)
  blebas.begin();
  blebas.write(_batteryLevel);

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

// get pressure reading from pressure sensor
float RocketelFS::readPressurePa() {
  return bmp.readPressure();
}

// read and return battery voltage
int RocketelFS::readBatteryLevel() {
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
    blebas.write(readBatteryLevel());
  else
    blebas.write(_batteryLevel);

  return _batteryLevel;
}
