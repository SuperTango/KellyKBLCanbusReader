/* Welcome to the ECU Reader project. This sketch uses the Canbus library.
It requires the CAN-bus shield for the Arduino. This shield contains the MCP2515 CAN controller and the MCP2551 CAN-bus driver.
A connector for an EM406 GPS receiver and an uSDcard holder with 3v level convertor for use in data logging applications.
The output data can be displayed on a serial LCD.

The SD test functions requires a FAT16 formated card with a text file of WRITE00.TXT in the card.


SK Pang Electronics www.skpang.co.uk

v3.0 21-02-11  Use library from Adafruit for sd card instead.

*/

#include <SdFat.h>        /* Library from Adafruit.com */
#include <SdFatUtil.h>
#include <TinyGPS.h>
#include <NewSoftSerial.h>
#include <KellyCanbus.h>
#include <mcp2515.h>
#include <stdlib.h>
#include <Time.h>


SdFat sd;
//Sd2Card card;
//SdVolume volume;
//SdFile root;
//ofstream file;
SdFile file;
SdFile rawFile;

NewSoftSerial lcdSerial = NewSoftSerial(3, 6);
NewSoftSerial gpsSerial = NewSoftSerial(4, 5);
KellyCanbus kellyCanbus = KellyCanbus(1.84);
#define TIMEZONEOFFSET -7
#define COMMAND 0xFE
#define CLEAR   0x01

#define KNOTSTOMPH 1.15077945
#define METERSTOMILES 0.000621371192
#define MILLISPERHOUR 3600000
#define BLANK_LINE "                    "

#define COMMA ","


#define GPSRATE 4800
//#define GPSRATE 38400

/* Define Joystick connection */
#define UP     A1
#define RIGHT  A2
#define DOWN   A3
#define CLICK  A4
#define LEFT   3
#define CHIP_SELECT 9

int LED2 = 8;
int LED3 = 7;

int brightness = 129;

//int baseChars16Column[4] = { 0, 64, 16, 80 };
int baseChars20Column[4] = { 0, 64, 20, 84 };

unsigned long currentMillis;
unsigned long lastFullReadMillis = 0;
unsigned long lastMillis2 = 0;
unsigned long lastClickMillis = 0;
unsigned long tDiffMillis;

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
float tripDistance;
//int year;
//uint8_t month, day, hour, minute, second, hundredths;
bool new_gps_data = false;

float milesPerKwh;
float whPerMile;

#define vOutPin A0
#define vInPin  A1
//int vInReading;
int vOutReading;
//float vIn;
#define vIn 5.0
float vOut;
#define Z1 1000.0
float z2;
float c;
//float f;

// store error strings in flash to save RAM
#define error(s) sd.errorHalt_P(PSTR(s))

uint32_t iterations = 0;
 
void setup() {
    uint16_t ret;
    Serial.begin(GPSRATE);
    lcdSerial.begin(9600);              /* Setup serial LCD and clear the screen */
    gpsSerial.begin(GPSRATE);
    pinMode(LED2, OUTPUT); 
    pinMode(LED3, OUTPUT); 

    digitalWrite(LED2, LOW);
    pinMode(UP,INPUT);
    pinMode(DOWN,INPUT);
    pinMode(LEFT,INPUT);
    pinMode(RIGHT,INPUT);
    pinMode(CLICK,INPUT);

    digitalWrite(UP, HIGH);       /* Enable internal pull-ups */
    digitalWrite(DOWN, HIGH);
    digitalWrite(LEFT, HIGH);
    digitalWrite(RIGHT, HIGH);
    digitalWrite(CLICK, HIGH);
  
    Serial.begin(115200);
    Serial.println("Kelly KBLI/HP Logger");  /* For debug use */
    Serial.print ( "Free Ram: " );
    Serial.println ( FreeRam() );
    
    analogReference(DEFAULT);
    setSyncProvider(gpsTimeToArduinoTime);

    clear_lcd();
    move_to ( 0, 0 );
    lcdSerial.print ( "MotoLogger Init" );
    move_to ( 2, 0 );
    lcdSerial.print ( "Free Ram: " );
    lcdSerial.print ( FreeRam() );
    move_to ( 1, 0 );
    lcdSerial.print ( "CAN Init " );
    if(kellyCanbus.init()) {
        lcdSerial.print("OK");
    } else {
        lcdSerial.print("Failed");
    } 
    delay ( 500 );
    move_to ( 1, 0 );

    init_logger();
    Serial.print ( "FreeRam: " ); Serial.println ( FreeRam() );

    clear_lcd();
    move_to ( 0, 0 );
    lcdSerial.print ( "MPH:" );
    move_to ( 1, 0 );
    lcdSerial.print ( "C:" );
    //move_to ( 1, 10 );
    //lcdSerial.print ( "i:" );
    move_to ( 2, 0 );
    lcdSerial.print ( "wm:" );
    move_to ( 2, 10 );
    lcdSerial.print ( "mk:" );
    move_to ( 3, 0 );
    lcdSerial.print ( "Trip:" );
}

void loop() {
    iterations++;
    currentMillis = millis();
    new_gps_data = false;
    time_t currentTime;
    while ( gpsSerial.available() ) {
        if ( new_gps_data = gps.encode( gpsSerial.read() ) ) {
            gps.f_get_position(&flat, &flon, &fix_age);
            fmph = gps.f_speed_mph();
            if ( fmph > 150 ) {
                fmph = 0;
            }
            //gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &fix_age);
            fcourse = gps.f_course();
            gps.get_datetime(&date, &time, &fix_age);
            if ( ( prev_flat != flat ) || ( prev_flon != flon ) ) {
              distance = gps.distance_between ( prev_flat, prev_flon, flat, flon ) * METERSTOMILES;
              if ( distance > 10 ) {
                  distance = 0;
              }
              tripDistance += distance;
              prev_flat = flat;
              prev_flon = flon;
            }
            break;
        }
    }

        /*
         * if we have new GPS data or it's been 1 second since the last 
         * update, perform the full update which includes reading all data,
         * updating the LCD, and writing a record to the primary file.
         */
    if ( ( new_gps_data ) || ( currentMillis - lastFullReadMillis > 1000 ) ) {

        kellyCanbus.fetchAllRuntimeData();
        tDiffMillis = currentMillis - lastFullReadMillis;
        if ( distance > 0 ) {
            whPerMile = ( kellyCanbus.wAvg * tDiffMillis / ( MILLISPERHOUR ) ) / distance;
            milesPerKwh = distance / ( kellyCanbus.wAvg * tDiffMillis / ( MILLISPERHOUR ) );
        } else {
            whPerMile = 0;
            milesPerKwh = 0;
        }


        //vInReading = analogRead(vInPin);
        vOutReading = analogRead(vOutPin);
        //vIn = 5.0 * (float)vInReading / 1024.0;
        vOut = vIn / 1024.0 * (float)vOutReading;
        z2 = ( -1 * vOut * Z1 ) / ( vOut - vIn );
        c = -2 * pow(10,-5) * pow( z2, 2)  + 0.1638 * z2 - 120.28;
        //f = c * 9 / 5 + 32;

        currentTime = now();

        move_to ( 0, 4 );
        lcdSerial.print ( "                " );
        move_to ( 0, 4 );
        lcdSerial.print ( fmph, 2 );
        move_to ( 0, 9 );
        lcdSerial.print ( kellyCanbus.getMPHFromRPM(), 1 );
        move_to ( 0, 14 );
        lcdSerial.print ( kellyCanbus.getTractionPackVoltage(), 1 );

        move_to ( 1, 2 );
        lcdSerial.print ( "        " );
        move_to ( 1, 2 );
        lcdSerial.print ( kellyCanbus.rawData[MOTOR_TEMP], DEC );
        move_to ( 1, 5 );
        lcdSerial.print ( c, 1 );
        move_to ( 1, 12 );
        lcdSerial.print ( "        " );
        //move_to ( 1, 12 );
        //lcdSerial.print ( kellyCanbus.count, DEC );
        move_to ( 1, 16 );
        lcdSerial.print ( ( kellyCanbus.getTractionPackVoltage() / 36 ), 2 );

        move_to ( 2, 3 );
        lcdSerial.print ( "       " );
        move_to ( 2, 3 );
        lcdSerial.print ( whPerMile, 2 );
        move_to ( 2, 13 );
        lcdSerial.print ( "       " );
        move_to ( 2, 13 );
        lcdSerial.print ( milesPerKwh, 2 );
        move_to ( 3, 5 );
        lcdSerial.print ( "      " );
        move_to ( 3, 5 );
        lcdSerial.print ( tripDistance, 2 );
        move_to ( 3, 12 );
        lcdPrintDigits ( hour ( currentTime ) );
            // odd, if starting before 3,14 and printing past 3,14, weird wrapping occurs.
        move_to ( 3, 14 );
        lcdSerial.print ( ":" );
        lcdPrintDigits ( minute ( currentTime ) );
        lcdSerial.print ( ":" );
        lcdPrintDigits ( second ( currentTime ) );

        file.print ( currentMillis, DEC ); file.print ( COMMA );
        file.print ( tDiffMillis, DEC ); file.print ( COMMA );
        file.print ( kellyCanbus.count, DEC ); file.print ( COMMA );
        file.print ( year ( currentTime ) ); filePrintDigits ( month( currentTime ) ); filePrintDigits ( day ( currentTime ) ); file.print ( COMMA );
        filePrintDigits ( hour ( currentTime ) ); filePrintDigits ( minute ( currentTime ) ); filePrintDigits ( second ( currentTime ) ); file.print ( COMMA );
        file.print ( fmph, 2 ); file.print ( COMMA );
        file.print ( kellyCanbus.getMPHFromRPM(), 2 ); file.print ( COMMA );
        file.print ( kellyCanbus.getTractionPackVoltage(), 3 ); file.print ( COMMA );
        file.print ( flat, 5 ); file.print ( COMMA );
        file.print ( flon, 5 ); file.print ( COMMA );
        file.print ( fcourse, 2 ); file.print ( COMMA );
        file.print ( distance, 5 ); file.print ( COMMA );
        file.print ( kellyCanbus.iAvg, 4 ); file.print ( COMMA );
        file.print ( kellyCanbus.vAvg, 4 ); file.print ( COMMA );
        file.print ( kellyCanbus.wAvg, 4 ); file.print ( COMMA );
        file.print ( whPerMile, 5 ); file.print ( COMMA );
        file.print ( milesPerKwh, 5 ); file.print ( COMMA );
        file.print ( c, 2 ); file.print ( COMMA );
        file.print ( z2, 5 ); file.print ( COMMA );

        for ( int i = 0; i < 22; i++ ) {
            file.print ( kellyCanbus.rawData[i], DEC );
            file.print ( COMMA );
        }
        file.println();
        lastFullReadMillis = currentMillis;
            // reset distance in case we do another round before getting another
            // set of GPS data, we don't re-calcualte wh/mi with a bogus distance.
        distance = 0;
        kellyCanbus.resetMotorInfo();

    } else {
        kellyCanbus.getCCP_A2D_BATCH_READ2();
    }
        /*
         * always write the raw current and voltage info to the raw file.
         */
    rawFile.print ( currentMillis, DEC );
    rawFile.print ( COMMA );
    for ( int i = CCP_A2D_BATCH_READ2_OFFSET; i < CCP_A2D_BATCH_READ2_OFFSET + 6; i++ ) {
        rawFile.print ( kellyCanbus.rawData[i], DEC );
        rawFile.print ( COMMA );
    }
    rawFile.println();

    if ( ( currentMillis - lastMillis2 ) >= 5000 ) {
        Serial.print ( "count: " );
        Serial.print ( kellyCanbus.count, DEC );
        Serial.print ( ", iterations: " );
        Serial.println ( iterations, DEC );
        iterations = 0;
        Serial.println ( "syncing..." );
        file.sync();
        rawFile.sync();
        lastMillis2 = currentMillis;
    }

    if (digitalRead(CLICK) == 0){  /* Check for Click button */
        file.println ( "Closing" );
        file.close();
        rawFile.close();
        Serial.println("Done");
        move_to ( 2, 0 );
        lcdSerial.print("DONE");
        while ( 1 ) {
        }
    } else if (digitalRead(UP) == 0) {  /* Check for Click button */
        if ( millis() - lastClickMillis > 1000 ) {
            lastClickMillis = millis();
            brightness += 5;
            if ( brightness > 157 ) {
                brightness = 157;
            }
            Serial.print ( "bright: " );
            Serial.println ( brightness, DEC );
            lcdSerial.print ( 0x7C, BYTE );
            lcdSerial.print ( brightness, BYTE );
        }
    } else if (digitalRead(DOWN) == 0) {  /* Check for Click button */
        if ( millis() - lastClickMillis > 1000 ) {
            lastClickMillis = millis();
            brightness -= 5;
            if ( brightness < 128 ) {
                brightness = 128;
            }
            Serial.print ( "bright: " );
            Serial.println ( brightness, DEC );
            lcdSerial.print ( 0x7C, BYTE );
            lcdSerial.print ( brightness, BYTE );
        }
    }

/*
 * NMEA Line:
 * $GPRMC,151731.000,A,3723.9826,N,12203.8878,W,0.85,204.01,070611,,*17,
 *
 * Current Log Line:
 * 11614_72844,0.00,0.00,0.000,3723.9890,N,12203.8761,W,0,68,xxx,xxx,xxx,xxx,xxx,xxx,xxx,xxx,xxx,xxx,xxx,xxx,xxx,xxx,xxx,xxx,xxx,xxx,xxx,xxx,xxx,xxx,
 */
}

void clear_lcd(void)
{
    lcdSerial.print(COMMAND,BYTE);
    lcdSerial.print(CLEAR,BYTE);
}  

void move_to ( int row, int column ) {
    int commandChar;
    commandChar = baseChars20Column[row];
    commandChar += column;
        /* set the high 7 bit to 1 per the spec */
    commandChar |= 0x80;
    lcdSerial.print(COMMAND,BYTE);
    lcdSerial.print(commandChar,BYTE);
}

void init_logger() {
    // initialize the SD card at SPI_HALF_SPEED to avoid bus errors with
    // breadboards.  use SPI_FULL_SPEED for better performance.
    // if SD chip select is not SS, the second argument to init is CS pin number
    if (sd.init(SPI_HALF_SPEED, CHIP_SELECT)) {
        Serial.println ( "sd init success" );
    } else {
        sd.initErrorHalt();
    }
    SdFile::dateTimeCallback(dateTime);
    move_to ( 1, 0 );
    lcdSerial.print ( "Log files...          " );

    Serial.println ( "Searching for files..." );
    // create a new file
    char name[] = "00000-LG.CSV";
    for (int i = 0; i <= 65530; i++) {
        if ( i == 65530 ) {
            clear_lcd();
            move_to ( 0,0 );
            Serial.println ( "No more filenames!" );
            lcdSerial.print ( "No more filenames!" );
            while ( 1 ) {
            }
        }
        name[0] = i/10000 + '0';
        name[1] = i%10000 / 1000 + '0';
        name[2] = i%1000 / 100 + '0';
        name[3] = i%100 / 10 + '0';
        name[4] = i%10 + '0';
        //Serial.print ( "checking" );
        //Serial.println ( name );
        if (sd.exists(name)) {
            continue;
        }
        if ( file.open ( name, O_WRITE | O_CREAT ) ) {
            break;
        } else {
            Serial.print ( "Failed opening file: " );
            Serial.println ( name );
            move_to ( 1, 0 );
            lcdSerial.print ( "Failed opening file" );
            error ( "file.open" );
        }
    }
    name[6] = 'R';
    name[7] = 'W';
    if ( ! rawFile.open ( name, O_WRITE | O_CREAT ) ) {
        Serial.print ( "Failed opening raw file: " );
        Serial.print ( name );
        move_to ( 1, 0 );
        lcdSerial.print ( "Failed opening file" );
        error ( "file.open" );
    }
    Serial.println ( "Successfully inited log files." );
    Serial.print ( "raw log file name: " );
    Serial.println ( name );
    move_to ( 1, 0 );
    lcdSerial.print ( "LogFile: " );
    lcdSerial.print ( name );
    delay ( 500 );
}

void dateTime(uint16_t* date, uint16_t* time) {
    time_t currentTime = now();
    // return date using FAT_DATE macro to format fields
    *date = FAT_DATE ( year ( currentTime ), month ( currentTime ), day ( currentTime ) );

    // return time using FAT_TIME macro to format fields
    *time = FAT_TIME ( hour ( currentTime ), minute ( currentTime ), second ( currentTime ) );
}

time_t gpsTimeToArduinoTime(){
  tmElements_t tm;
  int year;
  gps.crack_datetime(&year, &tm.Month, &tm.Day, &tm.Hour, &tm.Minute, &tm.Second, NULL, NULL);
    // ensure that the date is valid
  if ( year < 2011 ) {
      return 0;
  }
  tm.Year = year - 1970; 
  time_t time = makeTime(tm);
  return time + (TIMEZONEOFFSET * SECS_PER_HOUR);
}

void lcdPrintDigits(int digits){
  if ( digits < 10 ) {
      lcdSerial.print ( '0' );
  }
  lcdSerial.print ( digits );
}

void filePrintDigits(int digits){
  if ( digits < 10 ) {
      file.print ( '0' );
  }
  file.print ( digits );
}
