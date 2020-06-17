/*
  RocketelFE.cpp
  Library for Model Rocket Telemetry Project using Adafruit Feather nRF5280 Express
  Created by Neil Jacklin, June 15, 2020.
  MIT License.
*/

#include "Arduino.h"
#include "RocketelFE.h"

// globals --------------------------------------------------------------------
// some of these objects don't work as private variables :-(
Adafruit_FlashTransport_QSPI flashTransport;
Adafruit_SPIFlash flash(&flashTransport);
FatFileSystem fatfs;

// Constructor ----------------------------------------------------------------
RocketelFE::RocketelFE()
{
  // create flash and fatfs objects
  // Adafruit_FlashTransport_QSPI _flashTransport;
  // Adafruit_SPIFlash _flash(&_flashTransport);
  // FatFileSystem _fatfs;



  // create BMP sensor object
  Adafruit_BMP3XX _bmp; // I2C (with no arguments)


}

// init/begin -----------------------------------------------------------------
// returns true on success, false on failure
bool RocketelFE::begin()
{
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

  // init sensor
  if (!_bmp.begin()) {
    Serial.println(F("ERROR: Could not find a valid BMP3 sensor, check wiring!"));
    return false;
  }

  // Set up sensor configuration parameters
  // use BMP388 data sheet to see how to set parameters
  _bmp.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
  _bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  _bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  //bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  // first readings are no good, so burn off a few
  _bmp.performReading(); delay(100); _bmp.performReading();

  // BLE init

  // set name
  strcpy(_bleName,"RocketelFE-1");

  // init Bluefruit
  Bluefruit.begin();
  Bluefruit.setTxPower(RFE_BLE_TXPOWER_READ);
  Bluefruit.setName(_bleName);

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower(); // TODO remove?

  // bleDataOffset = Bluefruit.Advertising.count() + 2;
  // byte initData[] = { 0x00, 0xFA, 0x00, 0x00, 0xF0 };
  // Bluefruit.Advertising.addManufacturerData(&initData,5);

  // Bluefruit.Advertising.setStopCallback(adv_stop_callback);
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in units of 0.625 ms; 32 = 20.0 ms, 244 = 152.5 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);  // Stop advertising entirely after ADV_TIMEOUT seconds, 0 = never

  // TODO add BLE services and characteristics


  // set up user switch pin to digital input mode
  pinMode(PIN_USERSW,INPUT);

  // set up built-in LED for output
  pinMode(LED_BUILTIN, OUTPUT);

  // set mode to READ
  _mode = RFE_MODE_READ;

  // set initilized flag and exit
  _bInit = true;
  return _bInit;
}

// public methods -------------------------------------------------------------

uint32_t RocketelFE::getFlashJEDECID()
{
  return flash.getJEDECID();
}
