// Neil Jacklin
// Model Rocket Telemetry Mark 2 

// #defines
#define RFS_DEBUG 

// includes
#include <RocketelFS.h>

using R = RocketelFS; 
// Note: RocketelFS should be used as a static class--do not instantiate object

void setup() { // SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP 

  //// setup serial
  Serial.begin(115200);
  
  // try a few times to get Serial going, but move on if we can't get it
  for (int trySerial = 6; !Serial && trySerial > 0; trySerial--)
    delay(500);

  //// initialize sensor
  R::debug = true;
  R::init();

  Serial.print(F("RocketTel object was "));
  if ( !R.initialized() ) Serial.print(F("NOT "));
  Serial.println(F("intialized successfully."));

}

// Main Loop
// The loop logic should implement the timer-based functionality.  
// Command handling is taken care of by the BLE write callback functions.
void loop() { // LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP

  // switch based on mode
  switch ( R::getMode() )
  {
    case RFS_MODE_READ: // READ READ READ READ READ READ READ READ READ READ

      break;

    case RFS_MODE_WRITE: // WRITE WRITE WRITE WRITE WRITE WRITE WRITE WRITE

      break;

    case RFS_MODE_INIT:
      // try to re-init
      R::init();
      
      break;

    default:
      Serial.print(F("ERROR: Unrecognized mode: "));
      Serial.print(R::getMode());
      
  }
}
