// Neil Jacklin

#include <RocketelFE.h>

// rocket telemetry object
RocketelFE R;

void setup() {

  // setup serial
  Serial.begin(115200);
  
  // try a few times to get Serial going, but move on if we can't get it
  for (int trySerial = 6; !Serial && trySerial > 0; trySerial--)
    delay(500);

  // init sensor
  R.begin();

  Serial.print(F("R is initialized: "));
  Serial.println(R.initialized());


  if ( R.initialized() ) {
    Serial.print("Flash JEDEC ID: 0x");
    Serial.println(R.getFlashJEDECID(),HEX);
  }

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Looping...");
  delay(2000);
}
