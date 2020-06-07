/*********************************************************************
 This is an example for our nRF52 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

/* This sketch demonstrates the Bluefruit.Advertising API(). When powered up,
 * the Bluefruit module will start advertising for ADV_TIMEOUT seconds (by
 * default 30 seconds in fast mode, the remaining time slow mode) and then
 * stop advertising completely. The module will start advertising again if
 * PIN_ADV is grounded.
 */
#include <bluefruit.h>

#define PIN_ADV       PIN_BUTTON1
#define ADV_TIMEOUT   600 // seconds

void setup() 
{
  // configure PIN_ADV as input with a pullup (pin is active low)
  pinMode(PIN_ADV, INPUT_PULLUP);
  
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  Serial.println("Bluefruit52 Advanced Advertising Example");
  Serial.println("----------------------------------------\n");

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  Bluefruit.setName("Bluefruit52");

  // Set up and start advertising
  startAdv();

  Serial.println("Advertising is started"); 
}

int countOffset = 0;

void startAdv(void)
{   
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  //Bluefruit.Advertising.addName();

  // EDITED add manufacturing data
  countOffset = Bluefruit.Advertising.count() + 2;
//  Serial.print("DEBUG: count before adding mfgr data  = ");
//  Serial.println(countOffset);
 
  byte data[] = { 0x07, 0x07 };
  Bluefruit.Advertising.addManufacturerData(&data, 2);

  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html
   */
  Bluefruit.Advertising.setStopCallback(adv_stop_callback);
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in units of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(ADV_TIMEOUT);      // Stop advertising entirely after ADV_TIMEOUT seconds 
}

long lastAdvUpdate = 0;
uint8_t *advDataPtr;
uint16_t mfgrDataValue;

uint8_t dataBuffer[BLE_GAP_ADV_SET_DATA_SIZE_MAX];

uint16_t byteSwap16(uint16_t a) {
  return (a >> 8) | (a << 8);
}

void loop() 
{
  // Only check pin when advertising has already stopped
  if ( !Bluefruit.Advertising.isRunning() )
  {
    // Check if Pin is grounded
    if ( digitalRead(PIN_ADV) == 0 )
    {
      Bluefruit.Advertising.start(ADV_TIMEOUT);
      Serial.println("Advertising is started");
    }
  }

  // update advertising manufacturing data once per 
  if ( millis() - lastAdvUpdate >= 2000 ) {
    // latch time
    lastAdvUpdate = millis();
    
    // get advertising data
    advDataPtr = Bluefruit.Advertising.getData();

    memcpy(dataBuffer,advDataPtr,Bluefruit.Advertising.count());

    // print it out
    Serial.print("Debug adv data = [");
    for ( int i = 0; i < Bluefruit.Advertising.count(); i++ ) {
      Serial.print((uint8_t)advDataPtr[i],HEX);
      Serial.print(" ");
    }
    Serial.println("]");

    // parse out data
    memcpy(&mfgrDataValue,&advDataPtr[countOffset+0],2); 
    mfgrDataValue = byteSwap16(mfgrDataValue);

    Serial.print("mfgrdata Value = ");
    Serial.println(mfgrDataValue);
    
    // increment counter
    mfgrDataValue++;

    // update value in data buffer
    mfgrDataValue = byteSwap16(mfgrDataValue);
    dataBuffer[countOffset+0] = mfgrDataValue; 
    dataBuffer[countOffset+1] = mfgrDataValue >> 8; 

    // write advertising data
    Bluefruit.Advertising.setData(dataBuffer,Bluefruit.Advertising.count());
    
  }
}

/**
 * Callback invoked when advertising is stopped by timeout
 */
void adv_stop_callback(void)
{
  Serial.println("Advertising time passed, advertising will now stop.");
}
