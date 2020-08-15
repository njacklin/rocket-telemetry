// Neil Jacklin
// Model Rocket Telemetry Mark 2 

// #defines
#define RECORD_SENSOR_READ_RATE_MS  (100)
#define RECORD_FLASH_FLUSH_RATE_MS  (5000)
#define BATTERY_READ_RATE_MS      (10000)
#define UPDATE_BLE_TDS_RATE_MS    (2000)

// includes
#include <RocketelFS.h>

using R = RocketelFS; 
// Note: RocketelFS should be used as a static class--do not instantiate object


void setup() { // SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP 

  // setup serial
  Serial.begin(115200);
  
  // try a few times to get Serial going, but move on if we can't get it
  for (int trySerial = 3; !Serial && trySerial > 0; trySerial--)
    delay(500);

  // initialize sensor
  R::debug = true;
  R::init(); // TODO refactor init() to take in RECORD_SENSOR_READ_RATE_MS
             //      and initialize sensors accordingly

  Serial.print(F("RocketTel object was "));
  if ( !R::initialized() ) Serial.print(F("NOT "));
  Serial.println(F("intialized successfully."));

  // do some intial reads 
  R::readBattery();
  R::readAllSensors();

}

void loop() { // LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP

  // do mode-specific work
  switch ( R::getMode() )
  {
    case R::MODE_STANDBY: 
      
      // update BLE TDS
      if ( millis() - R::getLastBLETDSupdateTimeMs() >= UPDATE_BLE_TDS_RATE_MS ) {
        R::readAllSensors();
        R::updateBLETDS();
      }

      break;
      
    case R::MODE_READ: 
        // nothing to do
      break;

    case R::MODE_RECORD:
      
      if ( millis() - R::getLastSensorReadingTimeMs() >= RECORD_SENSOR_READ_RATE_MS ) {
        R::readAllSensors();
        R::writeFlashRecord();
      }

      if ( millis() - R::getLastFlashFlushTimeMs() >= RECORD_FLASH_FLUSH_RATE_MS ) {
        R::flushFlashWrites();
      }

      if ( millis() - R::getLastBLETDSupdateTimeMs() >= UPDATE_BLE_TDS_RATE_MS ) {
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
  if ( millis() - R::getLastBLETDSupdateTimeMs() >= UPDATE_BLE_TDS_RATE_MS ) {
    R::readBattery();
    R::updateBLEBatteryLevel();
  }
  
}
