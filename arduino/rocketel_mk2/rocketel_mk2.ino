// Neil Jacklin
// Model Rocket Telemetry Mark 2 

// #defines
#define RFS_DEBUG 

// includes
#include <RocketelFS.h>

// rocket telemetry object
RocketelFS R;

void setup() {

  // setup serial
  Serial.begin(115200);
  
  // try a few times to get Serial going, but move on if we can't get it
  for (int trySerial = 6; !Serial && trySerial > 0; trySerial--)
    delay(500);

  // init sensor
  R.begin();

  Serial.print(F("LOG: RocketTel object was "));
  if ( !R.initialized() ) Serial.print(F("NOT "));
  Serial.println(F("intialized successfully."));

}

void loop() {

  uint16_t timestampShortMs;
  float pressurePa;
  
  Serial.println("Looping...");

  pressurePa = R.readPressurePa();
  Serial.print("Pressure reading: ");
  Serial.print(pressurePa);
  Serial.println(" Pa");

  R.readBatteryLevel();
  Serial.print("Battery reading: ");
  Serial.print(R.getBatteryVoltage());
  Serial.print(" V, Level (0-100): ");
  Serial.print(R.getBatteryLevel());
  Serial.println();

  R.updateBLEBatteryLevel(false);

  // practice updating some BLE service characteristics

  switch ( R.getMode() ) {
    case RFS_MODE_INIT :
      R.bletds_mode_string.write("INIT");
      break;
    case RFS_MODE_READ : 
      R.bletds_mode_string.write("READ");
      break;
    case RFS_MODE_WRITE : 
      R.bletds_mode_string.write("WRITE");
      break; 
  }

  timestampShortMs = millis();
  R.bletds_timestamp_ms.notify(&timestampShortMs,2);
  
  R.bletds_pressure_pa.notify(&pressurePa,4);

  Serial.println();
  delay(2000);
}
