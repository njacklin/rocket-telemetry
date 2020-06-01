// Neil Jacklin
// 5/29/20 read all values, but no serial so that it executes on battery power

// for predefined board pins, refer to 
// ~/Library/Arduino15/packages/adafruit/hardware/nrf52/[ver]/variants/feather_nrf52840_express/variant.h

// battery voltage info
// from https://learn.adafruit.com/introducing-the-adafruit-nrf52840-feather/pinouts
// Default voltage range: 0-3.6V (uses the internal 0.6V reference with 1/6 gain)
// Default resolution: 12-bit (0..4096)
// Default mV per lsb (assuming 3.6V and 12-bit resolution): 1 LSB = 0.87890625 mV
// from variant.h, use PIN_VBAT as analog input to read battery level

#define BATT_VOLTAGE_LSB_mV (0.87890625)

// to get sensor data use I2C, since that is what the feather sense board uses
// need to connect SDI pin on feather express to SDA pin on BMP388 breakout board
// need to connect SCL (for I2C CLK) pin on feather express to SCK pin on BMP388 breakout board
// based on https://learn.adafruit.com/adafruit-bmp388/arduino
// code also copied from bmp3xx_simpletest

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

// the following define's are used only for SPI mode
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

// st sea level pressure in hPa for altitude calculations
#define SEALEVELPRESSURE_HPA (1013.25) // default/conical value
//#define SEALEVELPRESSURE_HPA (1011.30) // Mather Field, 11:27 PM Friday May 29 2020
// note: observed that default value gave more accurate altitude MSL than custom value!!
// check at https://www.aviationweather.gov/adds/metars/index?station_ids=KMHR&hoursStr=2&std_trans=translated&chk_metars=on

// instantiate sensor object in either I2C or SPI mode
Adafruit_BMP3XX bmp; // I2C
//Adafruit_BMP3XX bmp(BMP_CS); // hardware SPI
//Adafruit_BMP3XX bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK); 

#define DEBUG 

// forward delcare convertAltitude function
float convertAltitude(float seaLevelhPa, float pressurehPa); 

void setup() {

  // setup serial
  Serial.begin(115200);
  // while (!Serial); // if you do this, sketch will hang without Serial hookup (e.g. on battery!)

  // try a few times to get Serial going, but move on if we can't get it
  int trySerial = 10;
  while (!Serial && trySerial > 0)
  {
    trySerial--;
    delay(500);
  }
  
  Serial.println("Feather express test 2");
  Serial.println();

  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  // Set up sensor configuration parameters
  // use BMP388 data sheet to see how to set parameters
  bmp.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  //bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  
  // set up built-in LED for output
  pinMode(LED_BUILTIN, OUTPUT);

  // flash 3 times to indicate done with setup()
  int i;
  for (i = 0; i < 3; i++ )
  {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(500);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(500);                       // wait for a second
  }
  
}
 
void loop() {
  
  #ifdef DEBUG
  Serial.println("DEBUG: top of loop()");
  #endif
  
  // try to read sensor
  if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return; // no blink
  }

  // print sensor data
  Serial.print("Temperature = ");
  Serial.print(bmp.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bmp.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  // library readAltitude function takes another measurement; unnecessary
  // Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.print(convertAltitude(SEALEVELPRESSURE_HPA, bmp.pressure / 100.0));  
  Serial.println(" m");
  
  // blink LED
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(50);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(50);                       // wait for a second
}

// helper functions

// convert pressure reading to m, don't do another read
// see Adafruit_BMP3XX_Library/readAltitude for original
float convertAltitude(float seaLevelhPa, float pressurehPa) {
  return 44330.0 * (1.0 - pow(pressurehPa / seaLevelhPa, 0.1903));
}
