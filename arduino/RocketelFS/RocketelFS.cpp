/*
  RocketelFS.h
  Library for Model Rocket Telemetry Project using Adafruit Feather nRF5280 Sense
  Created by Neil Jacklin, June 17, 2020.
  MIT License.
*/

#include "Arduino.h"
#include "RocketelFS.h"

// globals --------------------------------------------------------------------
// some of these objects don't work as private variables :-(
Adafruit_FlashTransport_QSPI flashTransport;
Adafruit_SPIFlash flash(&flashTransport);
FatFileSystem fatfs;

Adafruit_BMP280 bmp; // I2C (with no arguments)

// Constructor ----------------------------------------------------------------
RocketelFS::RocketelFS()
{
  // nothing to do here
}

// init/begin -----------------------------------------------------------------
// returns true on success, false on failure
bool RocketelFS::begin()
{
  // PIN setup
  // set up user switch pin to digital input mode
  pinMode(PIN_USERSW,INPUT);
  // set up built-in LED for output
  pinMode(LED_BUILTIN, OUTPUT);
  // setup battery ADC PIN: none necessary

  // intitalize flash and mount FAT file system
  if (!flash.begin()) {
    Serial.println(F("ERROR: failed to initialize flash chip!"));
    return false;
  }

  if (!fatfs.begin(&flash)) {
    Serial.println(F("ERROR: failed to mount filesystem!"));
    Serial.println(F("ERROR: ensure chip formatted with the SdFat_format example"));
    return false;
  }

  // init pressure sensor
  if (!bmp.begin(RFS_I2C_ADDR_BMP280)) {
    Serial.println(F("ERROR: Could not find a valid BMP280 sensor, check wiring!"));
    return false;
  }

  bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,    /* Operating Mode. Normal reads every standby time. */
    Adafruit_BMP280::SAMPLING_X1,    /* Temp. oversampling */
    Adafruit_BMP280::SAMPLING_X4,    /* Pressure oversampling */
    Adafruit_BMP280::FILTER_X2,      /* IIR Filtering. */
    Adafruit_BMP280::STANDBY_MS_63); /* Standby time.  Should be less than real sampling time. */

  // first readings are no good, so burn off a few
  bmp.readPressure(); delay(100); bmp.readPressure();

  // set altitude settings for initial altitude algorithm
  changeAltitudeAlgorithm(_altitudeAlgorithm);

  // BLE init

  // init Bluefruit
  if (!Bluefruit.begin()) {
    Serial.println(F("ERROR: Could not begin() Bluefruit library."));
  }

  // set connect and disconnect callbacks
  Bluefruit.Periph.setConnectCallback(bleConnectCallback);
  Bluefruit.Periph.setDisconnectCallback(bleDisconnectCallback);

  // set Tx power and name
  Bluefruit.setTxPower(RFS_BLE_TXPOWER_READ);
  Bluefruit.setName(_bleName);
  // warning: Don't know if the above two lines need to be in that order.
  // I think each one adds something to the advertising data buffer.
  // TODO: look into docs/code to verify

  // add BLE services and characteristics

  // init BLE Device Information Service (DIS)
  // set some reasonable default values
  bledis.setManufacturer(_bleManufacturerStr);
  bledis.setModel(_bleModelStr);
  bledis.begin();

  // init BLE BAttery Service (BAS)
  blebas.begin();
  blebas.write(_batteryLevel);

  // BLE Telemetry Data Service
  bletds              = BLEService(UUID128_SVC_TDS);
  bletds.begin(); // must call service.begin() before adding any characteristics

  // TDS:timestamp_ms (uint32) characteristic
  bletds_timestamp_ms = BLECharacteristic(UUID128_CHR_TDS_TIMESTAMP_MS);
  bletds_timestamp_ms.setProperties(CHR_PROPS_NOTIFY);
  bletds_timestamp_ms.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  bletds_timestamp_ms.setFixedLen(4);
  bletds_timestamp_ms.begin();

  // TDS:timestamp_ms_str (utf8s) characteristic
  bletds_timestamp_ms_str = BLECharacteristic(UUID128_CHR_TDS_TIMESTAMP_MS_STR);
  bletds_timestamp_ms_str.setProperties(CHR_PROPS_NOTIFY);
  bletds_timestamp_ms_str.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  bletds_timestamp_ms_str.setMaxLen(20); // "Time(ms):#####"
  bletds_timestamp_ms_str.begin();

  // TDS:pressure_pa (uint32) characteristic
  bletds_pressure_pa  = BLECharacteristic(UUID128_CHR_TDS_PRESSURE_PA);
  bletds_pressure_pa.setProperties(CHR_PROPS_NOTIFY);
  bletds_pressure_pa.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  bletds_pressure_pa.setFixedLen(4);
  bletds_pressure_pa.begin();

  // TDS:pressure_pa_str (utf8s) characteristic
  bletds_pressure_pa_str  = BLECharacteristic(UUID128_CHR_TDS_PRESSURE_PA_STR);
  bletds_pressure_pa_str.setProperties(CHR_PROPS_NOTIFY);
  bletds_pressure_pa_str.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  bletds_pressure_pa_str.setMaxLen(20); // "P(Pa):######"
  bletds_pressure_pa_str.begin();

  // TDS:altitude_m (float) characteristic
  bletds_altitude_m  = BLECharacteristic(UUID128_CHR_TDS_ALTITUDE_M);
  bletds_altitude_m.setProperties(CHR_PROPS_NOTIFY);
  bletds_altitude_m.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  bletds_altitude_m.setFixedLen(4);
  bletds_altitude_m.begin();

  // TDS:altitude_str (utf8s) characteristic
  bletds_altitude_str  = BLECharacteristic(UUID128_CHR_TDS_ALTITUDE_STR);
  bletds_altitude_str.setProperties(CHR_PROPS_NOTIFY);
  bletds_altitude_str.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  bletds_altitude_str.setMaxLen(20); // "Alt(units)):######"
  bletds_altitude_str.begin();

  // TDS:max_altitude_m (float) characteristic
  bletds_max_altitude_m  = BLECharacteristic(UUID128_CHR_TDS_MAX_ALTITUDE_M);
  bletds_max_altitude_m.setProperties(CHR_PROPS_NOTIFY);
  bletds_max_altitude_m.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  bletds_max_altitude_m.setFixedLen(4);
  bletds_max_altitude_m.begin();

  // TDS:max_altitude_str (utf8s) characteristic
  bletds_max_altitude_str  = BLECharacteristic(UUID128_CHR_TDS_MAX_ALTITUDE_STR);
  bletds_max_altitude_str.setProperties(CHR_PROPS_NOTIFY);
  bletds_max_altitude_str.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  bletds_max_altitude_str.setMaxLen(20); // "MaxAlt(units)):######"
  bletds_max_altitude_str.begin();

  // TDS:mode_str (string) characteristic
  bletds_mode_str  = BLECharacteristic(UUID128_CHR_TDS_MODE_STR);
  bletds_mode_str.setProperties(CHR_PROPS_READ);
  bletds_mode_str.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  bletds_mode_str.setMaxLen(20); // "MODE:XXXXX"
  bletds_mode_str.begin();
  // bletds_mode_str.write("INIT"); // default value

  // BLE other services... TODO

  // set up BLE advertising
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  // according to Nordic docs, this flag advertises BLE only (explictly not BT classic)
  // Bluefruit.Advertising.addManufacturerData(&mfgrData,len);
  // Bluefruit.Advertising.setStopCallback(adv_stop_callback);
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in units of 0.625 ms; 32 = 20.0 ms, 244 = 152.5 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);  // Stop advertising entirely after ADV_TIMEOUT seconds, 0 = never

  // set mode to READ
  _mode = RFS_MODE_READ;
  bletds_mode_str.write("READ");

  // set initilized flag and exit
  _bInit = true;
  return _bInit;
}

// public methods -------------------------------------------------------------

// get flash (QPSI flash chip) ID
uint32_t RocketelFS::getFlashJEDECID()
{
  return flash.getJEDECID();
}

// read LASTLOGINFO.TXT
// return true if succeed (i.e. if it exists and was read successfully)
bool RocketelFS::readLastLogInfo()
{
  String lineStr;

  // try to open LASTLOG.TXT
  _file = fatfs.open("LASTLOG.TXT",FILE_READ);
  if (!_file)
    Serial.println(F("ERROR: could not open LASTLOG.TXT"));

  // read contents and store values in private variables
  lineStr = _file.readString();
  _lastLogIndex = lineStr.toInt();

  // get last number of records
  // TODO

  // if we get to the end, assume everything went okay
  return true;
}

// read pressure+temperature sensor, set relevant private variables
// note this includes setting altitude derived measurement
// returns pressure in pa as float
float RocketelFS::readPressureTempSensor() {
  // convention: set last sensor reading time immediately before read
  _lastSensorReadingTimeMs = _lastPressureTempSensorReadingTimeMs = millis();
  
  // record pressure and temp measurements
  _pressurePa = bmp.readPressure();
  _tempDegC = bmp.readTemperature();

  // compute altitude
  computeAltitude();

  return _pressurePa;
}

// change altitude algorithm
// this will include taking "initial"/ground measurements if needed
// returns false if there is an error
// TODO: extend signature to handle 2A/2B
bool RocketelFS::changeAltitudeAlgorithm(char *algorithmStr, bool resetMaxAlt) {
  // update offsets
  if ( strcmp(algorithmStr,"1A") == 0 ) {
    _altitudeOffsetM = 0.0f;
    _pressureOffsetPa = readPressureTempSensor();
  } else if  ( strcmp(algorithmStr,"1B") == 0 ) {
    _pressureOffsetPa = 101325.0f;
    _altitudeOffsetM = 
      44330.0f * (1.0f - pow(readPressureTempSensor() / _pressureOffsetPa,0.1903));
  // } else if  ( strcmp(algorithmStr,"2A") ) { // TODO
  // } else if  ( strcmp(algorithmStr,"2B") ) { // TODO
  } else {
    return false;
  }

  // update algorithm string state variable
  strcpy(_altitudeAlgorithm,algorithmStr);

  // reset max altitude
  if ( resetMaxAlt ) 
    _maxAltitudeM  = 0.0f;
  
  // recompute altitude
  computeAltitude();

  Serial.println(F("DEBUG: exiting changeAltitudeAlgorithm() successfully."));
  return true;
}

// read and return battery level
int RocketelFS::readBattery() {
  Serial.print(F("DEBUG: analogRead(PIN_BATTERYADC) = "));
  Serial.println(analogRead(PIN_BATTERYADC));
  _batteryVoltage = ADC_LSB_MV * (float)analogRead(PIN_BATTERYADC) / 1000.0f;
  _batteryLevel = _batteryVoltage / RFS_BATTERY_VOLTAGE_100PCT * 100.0f + 0.5f;
  _batteryLevel = constrain(_batteryLevel,0,100);

  return _batteryLevel;
}

// update BLE battery level advertised
// return battery level
// BUG: this always returns around 0.5V == 14% right now
int RocketelFS::updateBLEBatteryLevel(bool newMeasurement = true) {
  if (newMeasurement)
    blebas.write(readBattery());
  else
    blebas.write(_batteryLevel);

  return _batteryLevel;
}

// update all the characteristics of the BLE Telemetry Data Service
// the behavior of the method is affected by _mode 
void RocketelFS::updateBLETDS() {
  uint32_t timestampMs_uint32 = (uint32_t) _lastSensorReadingTimeMs;
  // TODO: think very carefully about what timestamp to report...
  uint32_t pressurePa_uint32 = (uint32_t) _pressurePa;

  // timestamp characteristics
  // TODO: move to mode WRITE only?  but it's useful...
  bletds_timestamp_ms.notify(&timestampMs_uint32,4);
  sprintf(_bleTimestampMsStr,"Time(ms):%d",timestampMs_uint32);
  bletds_timestamp_ms_str.notify(_bleTimestampMsStr);

  // pressure characteristics
  // TODO: move to mode WRITE only?  but it's useful...
  bletds_pressure_pa.notify(&pressurePa_uint32,4);
  sprintf(_blePressurePaStr,"P(Pa):%d",pressurePa_uint32);
  bletds_pressure_pa_str.notify(_blePressurePaStr);

  // altitude and max altitude characteristics
  bletds_altitude_m.notify(&_altitudeM,4);

  if ( strcmp(_altitudeStrUnits,"m") == 0 )
    sprintf(_bleAltitudeStr,"Alt(m):%.1f",_altitudeM);
  else if ( strcmp(_altitudeStrUnits,"ft") == 0 )
    sprintf(_bleAltitudeStr,"Alt(ft):%.1f",_altitudeM*RFS_CONVERT_M_TO_FT);
                                
  bletds_altitude_str.notify(_bleAltitudeStr);
  
  bletds_max_altitude_m.notify(&_maxAltitudeM,4);
  
  if ( strcmp(_altitudeStrUnits,"m") == 0 )
    sprintf(_bleMaxAltitudeStr,"MaxAlt(m):%.1f",_maxAltitudeM);
  else if ( strcmp(_altitudeStrUnits,"ft") == 0 )
    sprintf(_bleMaxAltitudeStr,"MaxAlt(ft):%.1f",_maxAltitudeM*RFS_CONVERT_M_TO_FT);
  
  bletds_max_altitude_str.notify(_bleMaxAltitudeStr);

  // mode-specific updates
  switch (_mode) {
    case RFS_MODE_READ:
      // update mode characteristic
      bletds_mode_str.write("MODE:READ");

      // TODO

      break;

    case RFS_MODE_WRITE:
      // update mode characteristic
      bletds_mode_str.write("MODE:WRITE");

      // TODO

      break;

    default:
      Serial.print(F("ERROR: Invalid mode in updateBLETDS(): "));
      Serial.println(_mode);
  }
}

// Callback invoked when a BLE connection is made
//   conn_handle connection where this event happens
void RocketelFS::bleConnectCallback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print(F("DEBUG: BLE Connected to "));
  Serial.println(central_name);
}

// Callback invoked when a BLE connection is dropped
//   conn_handle connection where this event happens
//   reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
void RocketelFS::bleDisconnectCallback(uint16_t conn_handle, uint8_t reason)
{
  // (void) conn_handle;
  // (void) reason;

  Serial.print(F("DEBUG: BLE Disconnected, reason = 0x"));
  Serial.println(reason, HEX);
}


// Create/open new log file.  
// This function will first create a LASTLOGINDEX.TXT, then LOGNN.META, 
// and then open a LOGNN.DAT for writing
bool RocketelFS::openNewLog() 
{
  char ones, tens;

  // find first available log index

  // try to open files LOGNN.DAT, increasing 
  //   NN from 00 to 99.  if all files are full, write to LOG99.DAT.
  strcpy(_filename,"LOGNN.DAT");
  for ( tens = '0'; tens <= '9'; tens++) 
  {
    _filename[3] = tens;
    for ( ones = '0'; ones <= '9'; ones++ ) 
    {
      _filename[4] = ones;
      
      if ( !fatfs.exists(_filename) )
        goto BREAKOUTFORFOR;
    }
  }
  BREAKOUTFORFOR:

  // filename now contains the first available log file name, "LOGNN.DAT"

  // set _currentLogIndex for future reference
  _currentLogIndex = 10*(tens - '0') + (ones - '0'); 

  // write LASTLOGINDEX.TXT with current log index
  fatfs.remove("LASTLOG.TXT"); // no overwrite mode, so delete if it exists
  _file = fatfs.open("LASTLOG.TXT",FILE_WRITE); // WRITE/APPEND
  if (!_file)
    Serial.println("ERROR: could not open LASTLOG.TXT");
  
  // write log index as chars
  _file.write(tens); 
  _file.write(ones); 
  _file.println();

  // close "LASTLOGINDEX.TXT"
  _file.close();

  // write LOGNN.META
  strcpy(&_filename[6],"META"); // change _filename to "LOGNN.META"
  fatfs.remove(_filename); // no overwrite mode, so delete if it exists
  _file = fatfs.open(_filename,FILE_WRITE); // WRITE/APPEND
  if (!_file) {
    Serial.print("ERROR: could not open file: ");
    Serial.println(_filename);
  }
  
  // write log index (redundant, but good integrity checking)
  _file.print("INDEX:"); 
  _file.write(tens); 
  _file.write(ones); 
  _file.println();

  // write record format
  _file.print("FORMAT:"); _file.println(RFS_RECORD_FORMAT);

  // write altitude reference
  _file.print("ALTREF:"); _file.println(_altitudeRef);

  // write altitude algorithm
  _file.print("ALTALGO:"); _file.println(_altitudeAlgorithm);
  // write pressure offset
  _file.print("P0:"); _file.println(_pressureOffsetPa);
  // write altitude offset
  _file.print("A0:"); _file.println(_altitudeOffsetM);

  // close "LOGNN.META"
  _file.close();

  // zero out num records written counter
  _numRecordsWritten = 0;

  // open LOGNN.DAT
  strcpy(&_filename[6],"DAT"); // change _filename to "LOGNN.DAT"
  return _file.open(_filename,FILE_WRITE);

}

// write record to flash file
// returns true on success and false on failure
bool RocketelFS::writeFlashRecord() 
{
  // for reference
  // format 1 = ( [uint16] timestamp_ms, [int16] pressure_pa - 100000, [int16] altitude_dm )

  uint16_t timestampmsData;
  int16_t pressureData;
  int16_t altitudedmData;

  if ( RFS_RECORD_FORMAT == 1 ) {
      
    // prepare data values for writing
    timestampmsData = _lastSensorReadingTimeMs;
    pressureData = _pressurePa - 100000.0f + 0.5f; // add 0.5 so that truncation does rounding
    altitudedmData = _altitudeM * 10.0f + 0.5f; // add 0.5 so that truncation does rounding

    // fill up record buffer
    _recordBuffer[0] = timestampmsData;
    _recordBuffer[1] = timestampmsData >> 8;
    _recordBuffer[2] = pressureData;
    _recordBuffer[3] = pressureData >> 8;
    _recordBuffer[4] = altitudedmData;
    _recordBuffer[5] = altitudedmData >> 8;
  }
  else {
    Serial.print(F("ERROR: in RocketelFS::writeFlashRecord(): unsupported RFS_RECORD_FORMAT = "));
    Serial.print(RFS_RECORD_FORMAT);
    Serial.println();
    return false;
  }

  // write record
  for (int i = 0; i < RFS_RECORD_BUFFER_BYTES; i++ )
    _file.write(_recordBuffer[i]);     

  // increment num record counter
  _numRecordsWritten++; 

  // exit 
  return true;  

}

// flush flash writes
// Files need to be closed to committed to flash (otherwise they are just in memory).
// Also, write number of records to LASTLOGINFO.TXT
// Returns true on success, false on failure.
bool RocketelFS::flushFlashWrites()
{
  // close .DAT file
  _file.close();

  _lastFlashFlushms = millis();

  // append num records to .META file
  strcpy(&_filename[6],"META"); // change _filename to "LOGNN.META"
  _file = fatfs.open(_filename,FILE_WRITE); // WRITE/APPEND
  // if (!_file) { // need to be fast, let's skip this for now
  //   Serial.print("ERROR: could not open file: ");
  //   Serial.println(_filename);
  // }
  
  // write log index (redundant, but good integrity checking)
  _file.print("NR:"); 
  _file.print(_numRecordsWritten); 
  _file.println();

  // close .META file
  _file.close();

  // reopen .DAT file
  strcpy(&_filename[6],"DAT"); // change _filename to "LOGNN.DAT"
  _file = fatfs.open(_filename, FILE_WRITE); // WRITE/APPEND 
       
}

// open LOGNN.DAT for reading
// return true on succes, false on failure
bool RocketelFS::openLogForRead(int logIndex)
{

  // form filename
  strcpy(_filename,"LOGNN.DAT");
  _filename[3] = '0' + (logIndex % 10);
  _filename[4] = '0' + (logIndex/10);

  Serial.print(F("DEBUG: Opening file for reading: "));
  Serial.println(_filename);

  // open file
  if ( fatfs.open(_filename, FILE_READ) )
    _currentLogIndex = logIndex;
    return true;
  else
    _currentLogIndex = -1;
    return false;

}

// close flash file
// return true on success, false on failure
bool RocketelFS::closeFlashFile() 
{
  return _file.close();
}

// private methods ------------------------------------------------------------

// compute altitude
// this will take the current _pressurePa, _altitudeAlgorithm, and related data,
// and compute the alititude measurement, and update maxAltitude if necessary.
// returns the computed altitude in m
float RocketelFS::computeAltitude() 
{
  // error checking
  if ( strcmp(_altitudeAlgorithm,"1A") == 0 && _altitudeOffsetM != 0.0f ) {
    Serial.println(F("WARNING: altitude offset not set right for algorithm 1A"));
    Serial.print(F("DEBUG: _altitudeOffsetM = ")); 
    Serial.println(_altitudeOffsetM);
  }
  
  if ( strcmp(_altitudeAlgorithm,"1B") == 0 && _pressureOffsetPa != 101325.0f ) { 
    Serial.println(F("WARNING: pressure offset not set right for algorithm 1B"));
  }

  // compute 
  // note: calculation is the same for all algorithms, just the offsets change
  _altitudeM = 44330.0f * (1.0f - pow(_pressurePa / _pressureOffsetPa, 0.1903f)) 
               - _altitudeOffsetM;

  // update max altitude if necessary
  if (_altitudeM > _maxAltitudeM)
    _maxAltitudeM = _altitudeM;
  
  // return altitude
  return _altitudeM;
}


