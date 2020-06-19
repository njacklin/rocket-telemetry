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
  // put your main code here, to run repeatedly:
  Serial.println("Looping...");

  Serial.print("Pressure reading: ");
  Serial.print(R.readPressurePa());
  Serial.println(" Pa");

  R.readBatteryLevel();
  Serial.print("Battery reading: ");
  Serial.print(R.getBatteryVoltage());
  Serial.print(" V, Level (0-100): ");
  Serial.print(R.getBatteryLevel());
  Serial.println();

  R.updateBLEBatteryLevel(false);

  Serial.println();
  delay(2000);
}
