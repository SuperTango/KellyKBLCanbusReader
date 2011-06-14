// A simple data logger for the Arduino analog pins with optional DS1307
// uses RTClib from https://github.com/adafruit/RTClib
#include <SdFat.h>
#include <SdFatUtil.h>  // define FreeRam()

#define CHIP_SELECT SS_PIN  // SD chip select pin
#define USE_DS1307       0  // set nonzero to use DS1307 RTC
#define LOG_INTERVAL  1000  // mills between entries
#define SENSOR_COUNT     3  // number of analog pins to log
#define ECHO_TO_SERIAL   1  // echo data to serial port if nonzero
#define WAIT_TO_START    1  // Wait for serial input in setup()
#define ADC_DELAY       10  // ADC delay for high impedence sensors

// file system object
SdFat sd;

// text file for logging
ofstream logfile;

// Serial print stream
ArduinoOutStream cout(Serial);

// buffer to format data - makes it eaiser to echo to Serial
char buf[80];
//------------------------------------------------------------------------------
#if SENSOR_COUNT > 6
#error SENSOR_COUNT too large
#endif  // SENSOR_COUNT
//------------------------------------------------------------------------------
// store error strings in flash to save RAM
#define error(s) sd.errorHalt_P(PSTR(s))
//------------------------------------------------------------------------------
#if USE_DS1307
// The Arduino IDE has a bug that causes Wire to be loaded even if USE_DS1307
// is false.  Comment out the next line to prevent Wire from being loaded.
#include <Wire.h>
// use RTClib from Adafruit
// https://github.com/adafruit/RTClib
#include "RTClib.h"
RTC_DS1307 RTC;  // define the Real Time Clock object
//------------------------------------------------------------------------------
// call back for file timestamps
void dateTime(uint16_t* date, uint16_t* time) {
    DateTime now = RTC.now();

  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(now.year(), now.month(), now.day());

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(now.hour(), now.minute(), now.second());
}
//------------------------------------------------------------------------------
// format date/time
ostream& operator << (ostream& os, DateTime& dt) {
  os << dt.year() << '/' << int(dt.month()) << '/' << int(dt.day()) << ' ';
  os << int(dt.hour()) << ':' << setfill('0') << setw(2) << int(dt.minute());
  os << ':' << setw(2) << int(dt.second()) << setfill(' ');
  return os;
}
#endif  // USE_DS1307
//------------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);

  // pstr stores strings in flash to save RAM
  cout << endl << pstr("FreeRam: ") << FreeRam() << endl;

#if WAIT_TO_START
  cout << pstr("Type any character to start\n");
  while (!Serial.available());
#endif  // WAIT_TO_START

#if USE_DS1307
  // connect to RTC
  Wire.begin();
  if (!RTC.begin()) error("RTC failed");

  // set date time callback function
  SdFile::dateTimeCallback(dateTime);
  DateTime now = RTC.now();
  cout  << now << endl;
#endif  // USE_DS1307

  // initialize the SD card at SPI_HALF_SPEED to avoid bus errors with
  // breadboards.  use SPI_FULL_SPEED for better performance.
  // if SD chip select is not SS, the second argument to init is CS pin number
  if (!sd.init(SPI_HALF_SPEED, CHIP_SELECT)) sd.initErrorHalt();

  // create a new file in root, the current working directory
  char name[] = "LOGGER00.CSV";

  for (uint8_t i = 0; i < 100; i++) {
    name[6] = i/10 + '0';
    name[7] = i%10 + '0';
    if (sd.exists(name)) continue;
    logfile.open(name);
    break;
  }
  if (!logfile.is_open()) error("file.open");

  cout << pstr("Logging to: ") << name << endl;

  // format header in buffer
  obufstream bout(buf, sizeof(buf));

  bout << pstr("millis");

#if USE_DS1307
  bout << pstr(",date time");
#endif  // USE_DS1307

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    bout << pstr(",sens") << int(i);
  }
  logfile << buf << endl;

#if ECHO_TO_SERIAL
  cout << buf << endl;
#endif  // ECHO_TO_SERIAL
}
//------------------------------------------------------------------------------
void loop() {
  uint32_t m;

  // wait for time to be a multiple of interval
  do {
    m = millis();
  } while (m % LOG_INTERVAL);

  // use buffer stream to format line
  obufstream bout(buf, sizeof(buf));

  // start with time in millis
  bout << m;

#if USE_DS1307
  DateTime now = RTC.now();
  bout << ',' << now;
#endif  // USE_DS1307

  // read analog pins and format data
  for (uint8_t ia = 0; ia < SENSOR_COUNT; ia++) {
#if ADC_DELAY
    analogRead(ia);
    delay(ADC_DELAY);
#endif  // ADC_DELAY
    bout << ',' << analogRead(ia);
  }
  bout << endl;

  // log data
  logfile << buf;

  // flush data to SD
  logfile.flush();

  // check for error
  if (!logfile) error("write data failed");

#if ECHO_TO_SERIAL
  cout << buf;
#endif  // ECHO_TO_SERIAL

  // don't log two points in the same millis
  if (m == millis()) delay(1);
}
