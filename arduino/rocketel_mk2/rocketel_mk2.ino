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

int loopCount = 0;
  
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
  R.readPressureTempSensor();

  R.updateBLETDS();

  if ( loopCount == 10 ) {
    R.setMaxAltitudeM(359.5f);
    Serial.print("DEBUG: Set max altitude = ");
    Serial.print(R.getMaxAltitudeM());
    Serial.println();
  }

  if ( loopCount == 11 ) {
    Serial.println("DEBUG: opening new log");
    R.openNewLog();
    
    Serial.print("DEBUG: R.currentLogIndex() = ");
    Serial.println(R.getCurrentLogIndex());
  }

  if ( loopCount >= 12 && loopCount <= 25 ) {
    Serial.println("DEBUG: writing record");
    R.writeFlashRecord();
  }

  if ( loopCount == 30 ) {
    Serial.println("DEBUG: flush flash writes");
    R.flushFlashWrites();
  }
  
  if ( loopCount == 32 ) {
    Serial.println("DEBUG: open file for reading");
    R.openLogForRead(7);
  }
    
  Serial.println();
  loopCount++;
  delay(500);
}
