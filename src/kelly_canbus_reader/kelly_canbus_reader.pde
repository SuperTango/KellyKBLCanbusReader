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
#define COMMAND 0xFE
#define CLEAR   0x01

#define KNOTSTOMPH 1.15077945
#define METERSTOMILES 0.000621371192
#define MILLISPERHOUR 3600000

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

uint32_t iterations;

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
unsigned long fix_age, speed, course, date, time;
unsigned long chars;
float flat, flon, fmph, fcourse;
float prev_flat, prev_flon;
float distance;
int year;
uint8_t month, day, hour, minute, second, hundredths;
bool new_gps_data;

float milesPerKwh;
float whPerMile;

// store error strings in flash to save RAM
#define error(s) sd.errorHalt_P(PSTR(s))
 
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
    lcdSerial.print ("logger OK         " );
    Serial.print ( "FreeRam: " ); Serial.println ( FreeRam() );

    iterations = 0;
}
 
void loop() {
    iterations++;

    while ( gpsSerial.available() ) {
        if ( new_gps_data = gps.encode( gpsSerial.read() ) ) {
            gps.f_get_position(&flat, &flon, &fix_age);
            fmph = gps.f_speed_mph();
            gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &fix_age);
            speed = gps.speed();
            fcourse= gps.f_course();
            gps.get_datetime(&date, &time, &fix_age);
            if ( ( prev_flat != flat ) || ( prev_flon != flon ) ) {
              distance = gps.distance_between ( prev_flat, prev_flon, flat, flon ) * METERSTOMILES;
              prev_flat = flat;
              prev_flon = flon;
            }
        }
    }
    
        /*
         * if we have new GPS data or it's been 1 second since the last 
         * update, perform the full update which includes reading all data,
         * updating the LCD, and writing a record to the primary file.
         */
    currentMillis = millis();
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

        move_to ( 0, 0 );
        lcdSerial.print ( fmph, 2 );
        lcdSerial.print ( " " );
        lcdSerial.print ( kellyCanbus.getMPHFromRPM(), 2 );
        lcdSerial.print ( " " );
        lcdSerial.print ( kellyCanbus.rawData[MOTOR_TEMP], DEC );
        lcdSerial.print ( "C" );
        move_to ( 1, 0 );
        lcdSerial.print ( "B+: " );
        lcdSerial.print ( kellyCanbus.getTractionPackVoltage(), 3 );
        move_to ( 2, 0 );
        lcdSerial.print ( "Wh/m: " );
        lcdSerial.print ( whPerMile, 2 );
        lcdSerial.print ( "   " );
        move_to ( 3, 0 );
        lcdSerial.print ( "m/kWh: " );
        lcdSerial.print ( milesPerKwh, 2 );
        lcdSerial.print ( " i " );
        lcdSerial.print ( kellyCanbus.count, DEC );

        file.print ( currentMillis, DEC );
        file.print ( COMMA );
        file.print ( tDiffMillis, DEC );
        file.print ( COMMA );
        file.print ( date, DEC );
        file.print ( COMMA );
        file.print ( time, DEC );
        file.print ( COMMA );
        file.print ( fmph, 2 );
        file.print ( COMMA );
        file.print ( kellyCanbus.getMPHFromRPM(), 2 );
        file.print ( COMMA );
        file.print ( kellyCanbus.getTractionPackVoltage(), 3 );
        file.print ( COMMA );
        file.print ( flat, 5 );
        file.print ( COMMA );
        file.print ( flon, 5 );
        file.print ( COMMA );
        file.print ( fcourse, 2 );
        file.print ( COMMA );
        file.print ( distance, 5 );
        file.print ( COMMA );
        file.print ( kellyCanbus.iAvg, 4 );
        file.print ( COMMA );
        file.print ( kellyCanbus.vAvg, 4 );
        file.print ( COMMA );
        file.print ( kellyCanbus.wAvg, 4 );
        file.print ( COMMA );
        file.print ( whPerMile, 5 );
        file.print ( COMMA );
        file.print ( milesPerKwh, 5 );
        file.print ( COMMA );

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
        Serial.println ( kellyCanbus.count, DEC );
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
        Serial.print ( "checking" );
        Serial.println ( name );
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
  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(year, month, day );

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(hour, minute, second );
}

