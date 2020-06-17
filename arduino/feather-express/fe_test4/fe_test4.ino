// Neil Jacklin
// 6/7/2020 feather board + bmp sensor + BLE test 
//
// Description: Start up in READ mode to help get data out of flash memory.
//   Long press goes into WRITE mode, stays in this mode until reset.
//   In WRITE mode, start a new file and write measurement data to file
//     file will be named LOG##.DAT, where ## is [00-99]
//   This version adds BLE.  Use "manufacturer data" to provide limited "human-readable hex" output.  
//   The data will be "## FA ## ## F[0|1|2]"
//     where the first byte (##) is the log number (last for read, current for write) ,
//     the next byte is 0xFA, fixed (F for file, then A for altitude next),
//     the next two bytes (## ##) are the altitude in feet, **offset by initial altitude**
//     the last byte is "F0" for INITIALIZING, "F1" for MODE_READ, or "F2" for MODE_WRITE
//   Human readable hex will work like this: 
//     Suppose the log number is 11.  Then the hex will read 0x11 so that to a human it is "11".
//     ex 2: altitude = 1000 ft.  Then the hex will read 0x1000.

// to get sensor data use I2C, since that is what the feather sense board uses
// need to connect SDI pin on feather express to SDA pin on BMP388 breakout board
// need to connect SCL (for I2C CLK) pin on feather express to SCK pin on BMP388 breakout board
// based on https://learn.adafruit.com/adafruit-bmp388/arduino
// code also copied from bmp3xx_simpletest


// PARAMS -----------------------------------------------------------------------

// at sea level pressure in hPa for altitude calculations
#define SEALEVELPRESSURE_HPA (1013.25) // default/conical value
//#define SEALEVELPRESSURE_HPA (1011.30) // Mather Field, 11:27 PM Friday May 29 2020
//#define SEALEVELPRESSURE_HPA (1007.7893) // Mather Field, 11:20 PM Friday May 29 2020
// note: observed that default value gave more accurate altitude MSL than custom value!!
// check at https://www.aviationweather.gov/adds/metars/index?station_ids=KMHR&hoursStr=2&std_trans=translated&chk_metars=on

// the data logging rate, i.e. the time between data record writes in ms 
#define LOGRATEMS 100L

// flush log file write rate
#define LOGFILEWRITERATEMS 5000L

// what defines a long button press in ms
#define LONGPRESSMS 2000L

// playback delay in milliseconds
#define PLAYBACKDELAYMS 10000L

// BLE params
char bleName[] = "Rocket-Tel-1";
#define BLEDATAUPDATEMS 2500L
#define BLETXPOWERREAD (2) 
#define BLETXPOWERWRITE (8)
#define BLEDATAOFFSETLOG 0
#define BLEDATAOFFSETALTMSB 2
#define BLEDATAOFFSETALTLSB 3
#define BLEDATAOFFSETMODE 4

// use this line (or comment it out) to control debug serial messages
#define DEBUG

// INIT -------------------------------------------------------------------------

#include <SPI.h>
#include <SdFat.h>
#include <Adafruit_SPIFlash.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <bluefruit.h>

// the following define's are used only for SPI mode
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

// On-board external flash (QSPI or SPI) macros should already
// defined in your board variant if supported
// - EXTERNAL_FLASH_USE_QSPI
// - EXTERNAL_FLASH_USE_CS/EXTERNAL_FLASH_USE_SPI
#if defined(EXTERNAL_FLASH_USE_QSPI)
  Adafruit_FlashTransport_QSPI flashTransport;
#elif defined(EXTERNAL_FLASH_USE_SPI)
  Adafruit_FlashTransport_SPI flashTransport(EXTERNAL_FLASH_USE_CS, EXTERNAL_FLASH_USE_SPI);
#else
  #error No QSPI/SPI flash are defined on your board variant.h !
#endif

Adafruit_SPIFlash flash(&flashTransport);

// file system object from SdFat
FatFileSystem fatfs;

// instantiate sensor object in either I2C or SPI mode
Adafruit_BMP3XX bmp; // I2C
//Adafruit_BMP3XX bmp(BMP_CS); // hardware SPI
//Adafruit_BMP3XX bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK); 

// user switch
#define PIN_USERSW PIN_BUTTON1

// some global vars
bool useBMPSensor = false;
int MODE = 0;
long lastWritems = 0L;
bool userSWpressed = false;
int buttonNotPressedValue = 0;
int buttonPressedValue = 1;
long buttonPressStartedms = 0;
char filename[64];
long dataCount = 0L;
long startedWaitingUserInputms = 0L;
bool printedReadIntro = false;
bool writeFileOpened = false;

File logFile, rootDir, child;

float altitudeBuffer = 0.0;
uint16_t timestampmsData = 0;
int16_t pressureData = 0;
uint16_t altitudedmData = 0;

float pressurePa;
float altitudeM;
float maxAltitudeM = 0.0;
uint16_t maxAltFt = 0;
float altitudeAtWriteStartM = 0.0;
                
byte rwBuffer[6];
long lastWriteFlushms = 0L;

uint8_t bleDataBuffer[BLE_GAP_ADV_SET_DATA_SIZE_MAX];
int bleDataOffset = 0;

char lastLogTensChar = 'F', lastLogOnesChar = 'F';
byte byteLog = 0xFF;
byte byteAltTH = 0xFF, byteAltTO = 0xFF;

bool bBleReadAdvert = false;
long lastBleUpdateMs = 0L;

float convertMtoFt = 3.28084;

// define the valid MODE values
#define MODE_READ (1)
#define MODE_WRITE (2)

// forward delcare helper functions
float convertAltitude(float seaLevelhPa, float pressurehPa); 
void unpackRecord(byte *b, uint16_t *timestampData, float *pressurePa, float *altitudeM);

// run-once code ------------------------------------------------------------
void setup() {
  
  // setup serial
  Serial.begin(115200);
  
  // try a few times to get Serial going, but move on if we can't get it
  int trySerial = 10;
  while (!Serial && trySerial > 0)
  {
    trySerial--;
    delay(500);
  }
  
  Serial.println(F("\n\nFeather express test 4 -------------------------------"));
  Serial.println();

  // initialize flash library and check its chip ID.
  if (!flash.begin()) {
    Serial.println(F("Error, failed to initialize flash chip!"));
    while(1) yield();
  }
  Serial.print(F("Flash chip JEDEC ID: 0x")); 
  Serial.println(flash.getJEDECID(), HEX);

  // call fatfs.begin() to mount the filesystem
  if (!fatfs.begin(&flash)) {
    Serial.println(F("ERROR: failed to mount filesystem!"));
    Serial.println(F("ERROR: ensure chip formatted with the SdFat_format example"));
    while(1) yield();
  }
  Serial.println("Mounted filesystem!");

  // init sensor
  if (bmp.begin()) {
    useBMPSensor = true;
  } 
  else {
    useBMPSensor = false; 
    
    Serial.println(F("WARNING: Could not find a valid BMP3 sensor, check wiring!"));
  }

  // Set up sensor configuration parameters
  // use BMP388 data sheet to see how to set parameters
  bmp.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  //bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  // first readings are no good, so burn off a few
  bmp.performReading();
  delay(100);
  bmp.performReading();

  // BLE init
  Bluefruit.begin();
  Bluefruit.setTxPower(BLETXPOWERREAD);
  Bluefruit.setName(bleName);
  
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  
  bleDataOffset = Bluefruit.Advertising.count() + 2;
  byte initData[] = { 0x00, 0xFA, 0x00, 0x00, 0xF0 }; 
  Bluefruit.Advertising.addManufacturerData(&initData,5);

//  Bluefruit.Advertising.setStopCallback(adv_stop_callback);
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in units of 0.625 ms; 32 = 20.0 ms, 244 = 152.5 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);      // Stop advertising entirely after ADV_TIMEOUT seconds, 0 = never

  // set up user switch pin to digital input mode
  pinMode(PIN_USERSW,INPUT);
  delay(50);
  buttonNotPressedValue = digitalRead(PIN_USERSW);
  buttonPressedValue = (int) !buttonNotPressedValue;
  #ifdef DEBUG
  Serial.print(F("DEBUG: buttonNotPressedValue = "));
  Serial.println(buttonNotPressedValue);
  Serial.print(F("DEBUG: buttonPressedValue = "));
  Serial.println(buttonPressedValue);
  #endif 

  // set up built-in LED for output
  pinMode(LED_BUILTIN, OUTPUT);

  // set state to READ
  MODE = MODE_READ;

} // end of setup()

// main loop code ------------------------------------------------------------
void loop() {
  // handle button input BUTTON BUTTON BUTTON BUTTON BUTTON BUTTON BUTTON BUTTON
  // long press is a one-way ticket to WRITE mode
  // anbd skip this check if we are in WRITE mode--focus on writing! 
  // (reset will get us back to READ)
  if ( MODE != MODE_WRITE ) { 
    
    if (!userSWpressed && digitalRead(PIN_USERSW)==buttonPressedValue) {
  
      buttonPressStartedms = millis();
      userSWpressed = true;

      // turn on LED to ack button push
      digitalWrite(LED_BUILTIN,HIGH);
      
      #ifdef DEBUG
      Serial.println(F("DEBUG: user sw press start detected!"));
      #endif 
      
    }
    else if (!userSWpressed && digitalRead(PIN_USERSW)==buttonNotPressedValue) {
      // do nothing, stay in unpressed state
      
      // turn off LED to nack button push
      digitalWrite(LED_BUILTIN,LOW);
      
      #ifdef DEBUG
      //Serial.println(F("DEBUG: userSWpressed == false and digitalRead(PIN_USERSW) == NOT PRESSED"));
      #endif 
    }
    else if (userSWpressed && digitalRead(PIN_USERSW)==buttonPressedValue) {

      // turn on LED to ack button push
      digitalWrite(LED_BUILTIN,HIGH);

      // if a "long press" has happened, then go to WRITE MODE
      if (millis() - buttonPressStartedms >= LONGPRESSMS) {
        // update mode
        MODE = MODE_WRITE;

        // print a status message
        Serial.println(F("*** SWITCHING TO WRITE MODE ***"));
        Serial.flush();
        
        // blink LED 10 times
        for ( int i = 0; i < 10; i++ )
        {
          digitalWrite(LED_BUILTIN, HIGH);
          delay(500);
          digitalWrite(LED_BUILTIN, LOW);
          delay(500);
        }
        
      }
    }
    else { // userSWpressed && digitalRead(PIN_USERSW)==buttonNotPressedValue
      userSWpressed = false;
      #ifdef DEBUG
      Serial.println(F("DEBUG: user sw unpressed"));
      #endif 
      
    } // end if-else if-else
    
  } // end if MODE

  // BLE advertising BLE BLE BLE BLE BLE BLE BLE BLE BLE BLE BLE BLE BLE BLE BLE BLE BLE BLE
  switch (MODE) { 
    case MODE_READ:

      if (bBleReadAdvert)
        break; // do nothing 

      // open directory
      rootDir = fatfs.open("/");
      if (!rootDir) {
        Serial.println(F("ERROR: failed to read flash file list."));
        break; // not as harsh as while(1) yield();
      }
      if (!rootDir.isDirectory()) {
        Serial.println(F("ERROR: expected \"\\\" to be a directory."));
        break; // not as harsh as while(1) yield();
      }

      child = rootDir.openNextFile();
      
      while (child) {
        child.getName(filename, sizeof(filename));
        
        // Record "last" filename
        if (!child.isDirectory()  && strncmp(filename,"LOG",3) == 0 ) {
            lastLogTensChar = filename[3];
            lastLogOnesChar = filename[4];
        }

        // Keep calling openNextFile to get a new file
        child = rootDir.openNextFile();
      }

      // close directory 
      rootDir.close();
      
      #ifdef DEBUG
      Serial.print(F("DEBUG: last log = "));
      Serial.print(lastLogTensChar); Serial.print(lastLogOnesChar);
      Serial.println();
      #endif 

      // get max altitude
      maxAltitudeM = 0.0;

      strcpy(filename,"LOGXX.DAT");
      filename[3] = lastLogTensChar;
      filename[4] = lastLogOnesChar;

      logFile = fatfs.open(filename,FILE_READ);
      
      while ( logFile.readBytes(rwBuffer,6) == 6) { // loop over data records

        unpackRecord(rwBuffer,&timestampmsData,&pressurePa,&altitudeM);
        
        if ( altitudeM > maxAltitudeM ) {
          maxAltitudeM = altitudeM;
        }
      }

      logFile.close();

      #ifdef DEBUG
      Serial.print(F("DEBUG: Max altitude found in file = "));
      Serial.print(maxAltitudeM*convertMtoFt);
      Serial.println(" ft");
      #endif

      // set BLE advertising data for read mode
      memcpy(bleDataBuffer,Bluefruit.Advertising.getData(),Bluefruit.Advertising.count());

      // set the first byte to last log number in "human-readable hex"
      bleDataBuffer[bleDataOffset+BLEDATAOFFSETLOG] = ( (lastLogTensChar - '0') << 4 ) 
                                                      | (lastLogOnesChar -  '0');

      // set the altitude bytes in "human-readable hex"
      maxAltFt = (maxAltitudeM * convertMtoFt + 0.5);
      bleDataBuffer[bleDataOffset+BLEDATAOFFSETALTMSB] = (uint8_t)(maxAltFt/1000) << 4; // set upper nibble of MSByte
      maxAltFt -= (maxAltFt/1000)*1000; // remove thousands decimal digit
      bleDataBuffer[bleDataOffset+BLEDATAOFFSETALTMSB] |= (uint8_t)(maxAltFt/100); // set lower nibble of MSByte
      maxAltFt -= (maxAltFt/100)*100; // remove hundreds decimal digit
      bleDataBuffer[bleDataOffset+BLEDATAOFFSETALTLSB] = (uint8_t)(maxAltFt/10) << 4; // set upper nibble of LSByte
      maxAltFt -= (maxAltFt/10)*10; // remove tens decimal digit
      bleDataBuffer[bleDataOffset+BLEDATAOFFSETALTLSB] |= (uint8_t)maxAltFt; // set lower nibble of LSByte
      maxAltFt -= maxAltFt; // will be 0 now... this could be omitted, but nice for consistency
      #ifdef DEBUG
      Serial.print(F("DEBUG: bytes in buffer for A = ["));
      Serial.print((uint16_t)bleDataBuffer[bleDataOffset+BLEDATAOFFSETALTMSB],HEX); 
      Serial.print(" ");
      Serial.print((uint16_t)bleDataBuffer[bleDataOffset+BLEDATAOFFSETALTLSB],HEX); 
      Serial.println("]");
      #endif
      
      bleDataBuffer[bleDataOffset+BLEDATAOFFSETMODE] = 0xF1;

      Bluefruit.Advertising.setData(bleDataBuffer,Bluefruit.Advertising.count());

      bBleReadAdvert = true;
      
      break;

    case MODE_WRITE:

      if ( millis() - lastBleUpdateMs >= BLEDATAUPDATEMS ) {
        // latch time
        lastBleUpdateMs = millis();

        // read in current advertising data
        memcpy(bleDataBuffer,Bluefruit.Advertising.getData(),Bluefruit.Advertising.count());

        // update log number to current log in "human-readable hex"
        bleDataBuffer[bleDataOffset+BLEDATAOFFSETLOG] = ( (filename[3] - '0') << 4 ) 
                                                        | (filename[4] - '0');

        // update altitude to max level seen in ft  in "human-readable hex"
        // TODO: (minus altitude at start of write)

        if ( maxAltitudeM - altitudeAtWriteStartM > 0.0 )
          maxAltFt = (( maxAltitudeM - altitudeAtWriteStartM) * convertMtoFt + 0.5);
        else
          maxAltFt = 0;
        
        bleDataBuffer[bleDataOffset+BLEDATAOFFSETALTMSB] = (uint8_t)(maxAltFt/1000) << 4; // set upper nibble of MSByte
        maxAltFt -= (maxAltFt/1000)*1000; // remove thousands decimal digit
        bleDataBuffer[bleDataOffset+BLEDATAOFFSETALTMSB] |= (uint8_t)(maxAltFt/100); // set lower nibble of MSByte
        maxAltFt -= (maxAltFt/100)*100; // remove hundreds decimal digit
        bleDataBuffer[bleDataOffset+BLEDATAOFFSETALTLSB] = (uint8_t)(maxAltFt/10) << 4; // set upper nibble of LSByte
        maxAltFt -= (maxAltFt/10)*10; // remove tens decimal digit
        bleDataBuffer[bleDataOffset+BLEDATAOFFSETALTLSB] |= (uint8_t)maxAltFt; // set lower nibble of LSByte
        maxAltFt -= maxAltFt; // will be 0 now... this could be omitted, but nice for consistency

        // update mode flag
        bleDataBuffer[bleDataOffset+BLEDATAOFFSETMODE] = 0xF2;

        // write advertising data
        Bluefruit.Advertising.setData(bleDataBuffer,Bluefruit.Advertising.count());
        
      }
        
      break;
  }

  // main mode functionality (READ = Present Serial Interface, WRITE = Write data)
  switch (MODE) {
  
    case MODE_READ: // READ READ READ READ READ READ READ READ READ READ READ

      if (Serial) {

        if (!printedReadIntro) {
          printedReadIntro = true;
  
          // list files 
          Serial.println(F("\n\nList of files:"));
        
          rootDir = fatfs.open("/");
          if (!rootDir) {
            Serial.println(F("ERROR: failed to read flash file list."));
            break; // not as harsh as while(1) yield();
          }
          if (!rootDir.isDirectory()) {
            Serial.println(F("ERROR: expected \"\\\" to be a directory."));
            break; // not as harsh as while(1) yield();
          }
  
          child = rootDir.openNextFile();
          
          while (child) {
            child.getName(filename, sizeof(filename));
            
            // Print the file name
            if (!child.isDirectory()) {
              Serial.print("- "); 
              Serial.print(filename);
              Serial.println();
            }
  
            // Keep calling openNextFile to get a new file
            child = rootDir.openNextFile();
          }

//          // If you want to list the files in the directory again call
//          // rewindDirectory().  Then openNextFile will start from the
//          // top again.
//          rootDir.rewindDirectory();

          // close directory
          rootDir.close();

          // solicit user input
          Serial.println();
          Serial.println(F("Enter \"[RP]XX\" to "));
          Serial.println(F("[C]SV file, [R]ead file, or [P]layback over Serial:"));

          // remember the time right now
          startedWaitingUserInputms = millis();
        }

        // if it has been too long, let's declare failure and redraw
        if ( millis() - startedWaitingUserInputms >= 20000L ) {
          while (Serial.read() > 0); // consume serial buffer gunk
          printedReadIntro = false;
        }

        // process input if there is serial input available
        if ( Serial.available() >= 3 ) 
        {
          
          char cmd = Serial.read();
          char charTens = Serial.read();
          char charOnes = Serial.read();

          // if there is extra data in serial buffer, read it and clean out buffer
          while ( Serial.read() > 0 );

          // build filename 
          strcpy(filename,"LOG##.DAT");
          filename[3] = charTens;
          filename[4] = charOnes;

          if ( cmd == 'C' || cmd == 'c' 
            || cmd == 'R' || cmd == 'r' 
            || cmd == 'P' || cmd == 'p' ) // CSV, Read, or Playback
          {
            Serial.print(F("Received command: "));
            if ( cmd == 'R' || cmd == 'r' )
              Serial.print(F("Read"));
            else if ( cmd == 'P' || cmd == 'p')
              Serial.print(F("Playback"));
            else if ( cmd == 'C' || cmd == 'c')
              Serial.print(F("CSV"));
            
            Serial.print(F(" for file "));
            Serial.println(filename);

            // open file
            logFile = fatfs.open(filename,FILE_READ);

            if ( cmd == 'R' || cmd == 'r' ) { // command == Read
              Serial.println(F("File read commencing..."));
            } else if ( cmd == 'P' || cmd == 'p' ) { // command == Playback
              Serial.print(F("\n\nPLAYBACK\n\n"));
              Serial.print(F("File playback will commence in "));
              Serial.print(PLAYBACKDELAYMS/1000);
              Serial.println(F(" seconds.  "));
              Serial.println(F("THIS IS A GOOD TIME TO SWITCH TO SERIAL PLOTTER."));
              Serial.println(F("AFTER PLAYBACK, PROGRAM WILL HANG UNTIL RESET."));
              Serial.flush();
              
              delay(PLAYBACKDELAYMS);
              
            } else if ( cmd == 'C' || cmd == 'c' ) { // commamnd == CSV
              
              Serial.println(F("Read back in CSV format..."));
              Serial.println();
              Serial.print(F("# data from flash memory file: "));
              Serial.println(filename);
              Serial.println(F("#time_ms,pressure_pa,altitude_m"));
              
            }

            // loop over each data record
            while ( logFile.readBytes(rwBuffer,6) == 6) { // loop over data records
              
              // unpack data values
              unpackRecord(rwBuffer,&timestampmsData,&pressurePa,&altitudeM);
            
              if ( cmd == 'R' || cmd == 'r' ) // do Read actions
              {
                #ifdef DEBUG // print read-write buffer contents
                Serial.print("DEBUG: rwBuffer = [");
                for (int i = 0; i < 6; i++) {
                  Serial.print(rwBuffer[i],HEX);
                  Serial.print(" ");
                }
                Serial.print("]");
                Serial.println();
                #endif

                // print out data record count
                Serial.print(F("Record "));
                Serial.print(dataCount++);

                Serial.print(F(", T (ms) = "));
                Serial.print(timestampmsData);

                Serial.print(F(", P (Pa) = "));
                Serial.print(pressurePa);

                Serial.print(F(", A (m) = "));
                Serial.print(altitudeM);
                Serial.println();
                                
              } else if ( cmd == 'P' || cmd == 'p' ) { // do Playback actions

//                Serial.print("timestamp(ms):");
//                Serial.print(timestampmsData); 
//                Serial.print(" ");
//                Serial.print("pressure(Pa):");
//                Serial.print(pressurePa);
//                Serial.print(" ");
                Serial.print("altitude(m):");
                Serial.print(altitudeM);
                Serial.println();

              } else if ( cmd == 'C' || cmd == 'c' ) { // end if Read else Playback else CSV
                
                Serial.print(timestampmsData); Serial.print(",");
                Serial.print(pressurePa); Serial.print(",");
                Serial.print(altitudeM); 
                
                Serial.println();
                
              }

            } // while data record

            // flush serial buffer
            Serial.flush();

            // close file
            logFile.close();

            // command closeout actions
            if ( cmd == 'R' || cmd == 'r' || cmd == 'C' || cmd == 'c'  ) { // Read or CSV
              Serial.println();
              Serial.println(F("File read complete."));

              printedReadIntro = false; // print out file list again on next loop
              
            } else if  ( cmd == 'P' || cmd == 'p' ) { // Playback
              
              // at the end of playback, hang the program so no more Serial data is sent
              while (1);
            }

         } else { // unrecognized command
              
           Serial.print(F("WARNING: Unrecognized command letter = "));
           Serial.println(cmd);
           
         } // end if recognized command else unrecognized command

       } // end if Serial.available() >= 3
       
     } else { // no Serial available -- just wait around for it to be plugged in
        
       delay(2000);
       Serial.begin(115200);
       delay(100);
     }
        
      break;

   case MODE_WRITE: // WRITE WRITE WRITE WRITE WRITE WRITE WRITE WRITE WRITE

     if ( !writeFileOpened )  {
         
       // open file
       // to figure out the file name, try to open files LOGNN.DAT, increasing 
       //   NN from 00 to 99.  if all files are full, write to LOG99.DAT.
       char ones, tens;
       strcpy(filename,"LOG##.DAT");
       for ( tens = '0'; tens <= '9'; tens++) 
       {
        filename[3] = tens;
        for ( ones = '0'; ones <= '9'; ones++ ) 
        {
          filename[4] = ones;
          #ifdef DEBUG 
          Serial.print(F("Checking file for existance: "));
          Serial.println(filename);
          #endif 
          
          if ( !fatfs.exists(filename) )
            goto BREAKOUTFORFOR;
        }
       }
       BREAKOUTFORFOR:
  
       // filename now contains the right file name
       
       Serial.print(F("Opening file for writing: "));
       Serial.println(filename);
       Serial.flush();

       // open the file
       logFile = fatfs.open(filename, FILE_WRITE); // WRITE/APPEND 
       // TODO: if file exists, it's not supposed to. should remove file before opening
       if (!logFile) {
         Serial.println(F("ERROR: failed to open log file!"));
         Serial.flush();
         return; // go back to top of loop()
       }
       Serial.println(F("Opened file log file for writing."));

       // init data count
       dataCount = 0;

       // zero maxAltitudeM
       maxAltitudeM = 0.0f;

       // init altitude
       if ( bmp.performReading() ) {
         altitudeAtWriteStartM = convertAltitude(SEALEVELPRESSURE_HPA, bmp.pressure / 100.0);
       } 
       // TODO: write first entry with timestamp = 0, P = ..., A = altitudeAtWriteStartM

       // set BLE tx power to WRITE level (higher so easily found)
       Bluefruit.setTxPower(BLETXPOWERWRITE);

       // make sure we don't do this again
       writeFileOpened = true;
     }

     // read and write data if the time is right
     if ( millis() - lastWritems >= LOGRATEMS ) {
       lastWritems = millis();


       // read sensor (or fail trying)
       if ( bmp.performReading() ) { 
         // turn on LED
         digitalWrite(LED_BUILTIN,HIGH);
       } else { // sensor reading went badly
         Serial.println("ERROR: Failed to perform reading :(");
         Serial.flush();
         return; 
        }

       // do write stuff
       
       altitudeBuffer = convertAltitude(SEALEVELPRESSURE_HPA, bmp.pressure / 100.0);

       if ( altitudeBuffer > maxAltitudeM )
         maxAltitudeM = altitudeBuffer;
       
       #ifdef DEBUG
//       Serial.print("Temperature (*C) = ");
//       Serial.print(bmp.temperature);
//       Serial.println();

       Serial.print("Timestamp (ms) = ");
       Serial.print(lastWritems);
       Serial.print(", ");
     
       Serial.print("Pressure (Pa) = ");
       Serial.print(bmp.pressure);
       Serial.print(", ");

       Serial.print("Altitude (m) = ");
       Serial.print(altitudeBuffer);  
       Serial.println();
       Serial.flush();
       #endif 

       // prepare data values for writing
       // for reference:
       // uint16_t timestampmsData 
       // int16_t pressureData 
       // uint16_t altitudedmData 
       timestampmsData = lastWritems;
       pressureData = bmp.pressure - 100000.0 + 0.5; // add 0.5 so that truncation does rounding
       altitudedmData = altitudeBuffer * 10.0 + 0.5; // add 0.5 so that truncation does rounding
       
       #ifdef DEBUG
       Serial.print("DEBUG: RECORD = ");
       Serial.println(dataCount++);
       Serial.print("DEBUG: DATA VALUES: t = ");
       Serial.print(timestampmsData); 
       Serial.print(" ["); Serial.print(timestampmsData,HEX); Serial.print("]");
       Serial.print(", p = "); Serial.print(pressureData);
       Serial.print(" ["); Serial.print(pressureData,HEX); Serial.print("]");
       Serial.print(", a = "); Serial.print(altitudedmData);
       Serial.print(" ["); Serial.print(altitudedmData,HEX); Serial.print("]");
       Serial.println();
       Serial.flush();
       #endif 

       // write data to file     

       rwBuffer[0] = timestampmsData;
       rwBuffer[1] = timestampmsData >> 8;
       rwBuffer[2] = pressureData;
       rwBuffer[3] = pressureData >> 8;
       rwBuffer[4] = altitudedmData;
       rwBuffer[5] = altitudedmData >> 8;

       #ifdef DEBUG
       Serial.print("DEBUG: rwBuffer is [");
       for (int i = 0; i < 6; i++ ) {
         Serial.print(rwBuffer[i],HEX);
         Serial.print(" ");
       }
       Serial.print("]");
       Serial.println();
       #endif

       for (int i = 0; i < 6; i++ )
         logFile.write(rwBuffer[i]);        

       // turn LED on
       digitalWrite(LED_BUILTIN,LOW);
       
     }

     // flush data if appropriate (partial writes don't seem to be saved)
     if ( millis() - lastWriteFlushms >= LOGFILEWRITERATEMS ) {
       lastWriteFlushms = millis();

       // close
       logFile.close();

       // reopen
       logFile = fatfs.open(filename, FILE_WRITE); // WRITE/APPEND 

       #ifdef DEBUG
       Serial.println("DEBUG: flushed file write.");
       #endif
       
     }

     break;

   default:
     Serial.print(F("Unrecognized MODE = "));
     Serial.println(MODE);
     delay(500);
  }

} // end of loop()


// helper functions ------------------------------------------------------

// convert pressure reading to m, don't do another read
// see Adafruit_BMP3XX_Library/readAltitude for original
float convertAltitude(float seaLevelhPa, float pressurehPa) {
  return 44330.0 * (1.0 - pow(pressurehPa / seaLevelhPa, 0.1903));
}

// read data record from flash file
// first arg should be the data buffer 
void unpackRecord(byte *b, uint16_t *timestampMs, float *pressurePa, float *altitudeM) {
  uint16_t t, a;
  int16_t p;
  
  // timestamp
  memcpy(&t,&b[0],2);
  *timestampMs = t;

  // pressure
  memcpy(&p,&b[2],2);
  *pressurePa = (float)p + 100000.0;

  // altitude 
  memcpy(&a,&b[4],2);
  *altitudeM = (float)a / 10.0;

  return;
}
