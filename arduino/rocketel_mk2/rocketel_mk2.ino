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

  Serial.println("Looping...");

  // battery stuff
  R.readBattery();
  Serial.print("Battery reading: ");
  Serial.print(R.getLastBatteryVoltage());
  Serial.print(" V, Level (0-100): ");
  Serial.print(R.getLastBatteryLevel());
  Serial.println();

  R.updateBLEBatteryLevel(false);

  // practice new method calls
  R.readPressureSensor();

  R.updateBLETDS();

  Serial.println();
  delay(2000);
}
