// simple example from 
// https://learn.adafruit.com/introducing-the-adafruit-nrf52840-feather/arduino-board-setup

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  // setup serial
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Feather board check");
  Serial.println();

  // flash LED 3 times to indicate done with setup()
  int i;
  for (i = 0; i < 3; i++ )
  {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(500);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(500);                       // wait for a second
    Serial.println("blink");
  }
  
}
 
void loop() {
  
  Serial.println("top of loop");
  
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}
