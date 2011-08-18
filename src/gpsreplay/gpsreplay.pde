#include <TinyGPS.h>

#include <SdFat.h>        /* Library from Adafruit.com */
#include <NewSoftSerial.h>

#define BUFFSIZ 90
#define GPSRATE 4800
#define CHIP_SELECT 9

#define COMMAND 0xFE
#define CLEAR   0x01
#define CLICK  A4
#define METERSTOMILES 0.000621371192
SdFat sd;
SdFile file;
SdFile file2;
SdFile rawFile;

int baseChars20Column[4] = { 0, 64, 20, 84 };
NewSoftSerial gpsSerial = NewSoftSerial(4, 5);
NewSoftSerial lcdSerial = NewSoftSerial(3, 6);
unsigned long lastMillis = 0;

char buffer[BUFFSIZ];
char buffidx;
#define error(s) sd.errorHalt_P(PSTR(s))

/*
 * Vars needed by tinyGPS
 */
TinyGPS gps;
long lat, lon;
unsigned long fix_age, course, date, time;
unsigned long chars;
float flat, flon, fmph, fcourse;
float prev_flat, prev_flon;
float distance;
int year;
uint8_t month, day, hour, minute, second, hundredths;
bool new_gps_data;

float milesPerKwh;
float whPerMile;

void setup() {
    lcdSerial.begin(9600);              /* Setup serial LCD and clear the screen */
    gpsSerial.begin(GPSRATE);
    Serial.begin ( 115200 );
    init_logger();
    pinMode(CLICK,INPUT);
    digitalWrite(CLICK, HIGH);
    Serial.println ( "Starting loop" );
}

void loop() {
    uint16_t c;
    while ( c = file.read() ) {
        if ( c == -1 ) {
            file2.close();
            Serial.println ( "Done loop" );
            while ( 1 ) {
            }
        }
        if ( new_gps_data = gps.encode( byte(c) ) ) {
            gps.f_get_position(&flat, &flon, &fix_age);
            fmph = gps.f_speed_mph();
            gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &fix_age);
            fcourse = gps.f_course();
            gps.get_datetime(&date, &time, &fix_age);
            if ( ( prev_flat != flat ) || ( prev_flon != flon ) ) {
                distance = gps.distance_between ( prev_flat, prev_flon, flat, flon ) * METERSTOMILES;
                prev_flat = flat;
                prev_flon = flon;
                file2.print ( flat, 5 );
                file2.print ( "," );
                file2.print ( flon, 5 );
                file2.print ( "," );
                file2.print(distance, 5 );
                file2.print ( "," );
                file2.print(fmph, 5 );
                file2.print ( "," );
                file2.print(fcourse, 5 );
                file2.println();
            } else {
                distance = 0;
            }
        }
    }
}


/*
bool readline(void) {
  char c;
  int available;
  
  buffidx = 0; // start at begninning
  if ( ! gpsSerial.available() ) {
      return false;
  }
  while (1) {
      c=gpsSerial.read();
      if (c == -1)
        continue;
      if (c == '\r')
        continue;
      if ((buffidx == BUFFSIZ-1) || (c == '\n')) {
        buffer[buffidx] = 0;
        return true;
      }
      buffer[buffidx++]= c;
  }
}
*/


void init_logger() {
    // initialize the SD card at SPI_HALF_SPEED to avoid bus errors with
    // breadboards.  use SPI_FULL_SPEED for better performance.
    // if SD chip select is not SS, the second argument to init is CS pin number
    if (sd.init(SPI_HALF_SPEED, CHIP_SELECT)) {
        Serial.println ( "sd init success" );
    } else {
        sd.initErrorHalt();
    }
    //SdFile::dateTimeCallback(dateTime);
    //move_to ( 1, 0 );
    //lcdSerial.print ( "Log files...          " );

    Serial.println ( "Searching for files..." );
    // create a new file
    char name[] = "00045-LG.NMA";
    if ( ! file.open ( name, O_READ ) ) {
        Serial.print ( "Failed opening file: " );
        Serial.println ( name );
        error ( "file.open" );
    }

    char name2[] = "00045-RA.CSV";
    if ( ! file2.open ( name2, O_WRITE | O_CREAT ) ) {
        Serial.print ( "Failed opening file2: " );
        Serial.println ( name );
        error ( "file2.open" );
    }
    Serial.println ( "Done" );
}
/*
void clear_lcd(void)
{
    lcdSerial.print(COMMAND,BYTE);
    lcdSerial.print(CLEAR,BYTE);
}  

void move_to ( int row, int column ) {
    int commandChar;
    commandChar = baseChars20Column[row];
    commandChar += column;
        // set the high 7 bit to 1 per the spec 
    commandChar |= 0x80;
    lcdSerial.print(COMMAND,BYTE);
    lcdSerial.print(commandChar,BYTE);
}

*/
