// Neil Jacklin
// Model Rocket Telemetry Mark 2 

// #defines
#define WRITE_MODE_SENSOR_READ_RATE_MS  (100)
#define READ_BATTERY_RATE_MS            (10000)
#define UPDATE_BLE_TDS_RATE_MS          (2000)

// includes
#include <RocketelFS.h>

using R = RocketelFS; 
// Note: RocketelFS should be used as a static class--do not instantiate object

unsigned long lastBatteryReadMs = 0L;
unsigned long lastBLETDSUpdateMs = 0L;


void setup() { // SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP 

  //// setup serial
  Serial.begin(115200);
  
  // try a few times to get Serial going, but move on if we can't get it
  for (int trySerial = 10; !Serial && trySerial > 0; trySerial--)
    delay(500);

  //// initialize sensor
  R::debug = true;
  R::init(); // TODO refactor init() to take in WRITE_MODE_SENSOR_READ_RATE_MS
             //      and initialize sensors accordingly

  Serial.print(F("RocketTel object was "));
  if ( !R::initialized() ) Serial.print(F("NOT "));
  Serial.println(F("intialized successfully."));

  // do some intial reads 
  lastBatteryReadMs = millis();
  R::readBattery();
  
  R::readAllSensors();

}

void loop() { // LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP

  // do mode-specific work (except BLE functions)
  switch ( R::getMode() )
  {
    case RFS_MODE_READ: 
      break;

    case RFS_MODE_WRITE: 

      break;

    case RFS_MODE_INIT: 
      // try to re-init
      R::init();
      
      break;

    default:
      Serial.print(F("ERROR: Unrecognized mode: "));
      Serial.print(R::getMode());
      
  }

  // read battery and update BLE battery service
  if ( millis() - lastBatteryReadMs >= UPDATE_BLE_TDS_RATE_MS ) {
    R::updateBLEBatteryLevel(true);
    lastBatteryReadMs = millis();
  }

  // update BLE TDS
  if ( millis() - lastBLETDSUpdateMs >= UPDATE_BLE_TDS_RATE_MS ) {
    updateBLETDS();
  }

  
}
