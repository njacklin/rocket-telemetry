// feather_flashfs_utility
// Utility sketch to read flash fs file system.
// Read back text files.
// Instead of implementing erase all or format commands,
//   it is recommended to run Aafruit SPIFlash SdFat_format.ino instead.


// INIT -------------------------------------------------------------------------

#include <SPI.h>
#include <SdFat.h>
#include <Adafruit_SPIFlash.h>

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

File file;

char filename[64];

long startedWaitingUserInputms = 0L;
bool printedIntro = false;

void setup() { // SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP 
  
  // setup serial
  Serial.begin(115200);
  
  // try a few times to get Serial going, but move on if we can't get it
  int trySerial = 10;
  while (!Serial && trySerial > 0)
  {
    trySerial--;
    delay(500);
  }
  
  Serial.println(F("\n\nFeather Flash FS File Reader Utility -------------------"));
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


}

bool cmdResult;

void loop() { // LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP 

  if ( Serial ) {
  
    if (!printedIntro) {
  
      // list files 
      Serial.print(F("\n\n\nList of files:\n\n"));
      fatfs.ls(LS_SIZE); // () for file names only, (LS_SIZE) for size and file names
      
      // solicit user input
      Serial.println();
      Serial.println(F("Enter \"[TBD]:FILENAME\" to "));
      Serial.println(F("[T] Read Text file, [B] Read Binary file, [D] Delete file"));
      Serial.println();

      // set printed intro flag
      printedIntro = true;
  
      // remember the time right now
      startedWaitingUserInputms = millis();
    }
  
    // if it has been too long, let's declare failure and redraw
    if ( millis() - startedWaitingUserInputms >= 20000L ) {
      while (Serial.read() > 0); // consume serial buffer gunk
      printedIntro = false;
    }
  
    // process input if there is serial input available
    if ( Serial.available() >= 1 ) 
    {
      char cmd = Serial.read();
      Serial.read(); // throw away ":" character
      String filenameStr = Serial.readString();
      strcpy(filename,filenameStr.c_str());
      if ( filename[strlen(filename)-1] == '\n' )
        filename[strlen(filename)-1] = '\0'; // replace trailing '\n' with '\0'
  
      // if there is extra data in serial buffer, read it and clean out buffer
      while ( Serial.read() > 0 );
      
      if ( cmd == 'T' || cmd == 'B'  ) // one of the read commands
      {
        Serial.print(F("Received command: "));
        Serial.println(cmd);
        
        Serial.print(F("  for file: '"));
        Serial.print(filename);
        Serial.println("'");
  
        // open file
        file = fatfs.open(filename,FILE_READ);

        if (!file) {
          Serial.println(F("ERROR: Cannot open file for reading"));
        }

        // read file 
        if ( cmd == 'T' ) {
          
          while ( file.available() ) {
            Serial.println( file.readString() );
          }
            
        } else if (cmd == 'B') {
          Serial.println(F("WARNING: binary read not yet implemented")); // TODO
        }
  
        // flush serial buffer
        Serial.flush();
  
        // close file
        file.close();
        
        // command closeout actions
        Serial.println();
        Serial.println(F("File read complete."));
      
        printedIntro = false; // print out file list again on next loop
  
     } else if ( cmd == 'D' )  {
      
       Serial.print(F("Deleting file: '"));
       Serial.print(filename);
       Serial.println("'");

       cmdResult = fatfs.remove(filename);

       if (cmdResult) 
         Serial.println(F("File deleted successfully."));
       else
         Serial.println(F("File delete FAILED!"));

       printedIntro = false; // print out file list again on next loop

     } else { // unrecognized command
          
       Serial.print(F("WARNING: Unrecognized command letter = "));
       Serial.println(cmd);
       
     } // end if recognized command else unrecognized command
  
   } // end if Serial.available() >= 1
   
  } else { // no Serial available -- just wait around for it to be plugged in
    
   delay(2000);
   Serial.begin(115200);
   delay(100);
  }

}
