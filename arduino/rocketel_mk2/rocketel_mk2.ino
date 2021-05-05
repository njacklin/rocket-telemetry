// Neil Jacklin
// Model Rocket Telemetry Mark 2 

// #defines
#define RECORD_SENSOR_READ_RATE_MS (100L)
#define RECORD_FLASH_FLUSH_RATE_MS (5000L)
#define UPDATE_BLE_BAS_RATE_MS (10000L)
#define UPDATE_BLE_TDS_RATE_MS (2000L)

// includes
#include "RocketelFS.h"

using R = RocketelFS; 
// Note: RocketelFS should be used as a static class--do not instantiate object

void setup() { // SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP 

  // setup serial
  Serial.begin(115200);
  
  // try a few times to get Serial going, but move on if we can't get it
  for (int trySerial = 10; !Serial && trySerial > 0; trySerial--)
    delay(500);

  // print ASCII art
  Serial.println();
  Serial.println(F("   ___           __       __      __________  __  _____     ___ "));
  Serial.println(F("  / _ \\___  ____/ /_____ / /____ / / __/ __/ /  |/  / /__  |_  |"));
  Serial.println(F(" / , _/ _ \\/ __/  '_/ -_) __/ -_) / _/_\\ \\  / /|_/ /  '_/ / __/ "));
  Serial.println(F("/_/|_|\\___/\\__/_/\\_\\__/\\__/\\__/_/_/ /___/ /_/  /_/_/\\_\\ /____/ "));
  Serial.println();
          
  // set up built-in LED for output
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_BUILTIN, LOW); // off

  // initialize sensor
  R::debug = true;
  R::init(); // TODO refactor init() to take in RECORD_SENSOR_READ_RATE_MS
             //      and initialize sensors accordingly

  Serial.print(F("RocketTel object was "));
  if ( !R::initialized() ) Serial.print(F("NOT "));
  Serial.println(F("intialized successfully."));

  // list all files on flash
  R::listAllFiles();

  // do some intial reads 
  R::readBattery();
  R::readAllSensors();

}

void loop() { // LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP

  // do mode-specific work
  switch ( R::getMode() )
  {
    case R::MODE_STANDBY: 
    
      digitalWrite(LED_BUILTIN,LOW);
      
      // update BLE TDS
      if ( millis() - R::getLastBLETDSUpdateTimeMs() >= UPDATE_BLE_TDS_RATE_MS ) {
        R::readAllSensors();
        R::updateBLETDS();
      }

      break;
      
    case R::MODE_READ: 
      digitalWrite(LED_BUILTIN,LOW);
        
      break;

    case R::MODE_RECORD:
      
      if ( millis() - R::getLastSensorReadingTimeMs() >= RECORD_SENSOR_READ_RATE_MS ) {
          
        R::readAllSensors();
        R::writeFlashRecord();

        if (digitalRead(LED_BUILTIN) == LOW)
          digitalWrite(LED_BUILTIN,HIGH);
        else
          digitalWrite(LED_BUILTIN,LOW);
      }

      if ( millis() - R::getLastFlashFlushTimeMs() >= RECORD_FLASH_FLUSH_RATE_MS ) {
        R::flushFlashWrites();
      }

      if ( millis() - R::getLastBLETDSUpdateTimeMs() >= UPDATE_BLE_TDS_RATE_MS ) {
        R::updateBLETDS();
      }

      break;

    case R::MODE_INIT:
      // try to re-init
      R::init();
      
      break;

    default:
      Serial.print(F("ERROR: Unrecognized mode: "));
      Serial.print(R::getMode());
      
  }

  // read battery and update BLE battery service
  if ( millis() - R::getLastBLEBASUpdateTimeMs() >= UPDATE_BLE_BAS_RATE_MS ) {
      
    R::readBattery();
    R::updateBLEBAS();
  }
  
}
