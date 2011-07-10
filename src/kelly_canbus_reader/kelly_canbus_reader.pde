#include <SdFat.h>
#include <SdFatUtil.h>
#include <TinyGPS.h>
#include <NewSoftSerial.h>
#include <KellyCanbus.h>
#include <Time.h>
#include <avr/pgmspace.h>

#define STATE_GATHERDATA 1
#define STATE_TRANSFERFILES 2


SdFat sd;
SdFile logFile;
SdFile rawFile;
SdFile readFile;

//Print *stream = &Serial;
Print *stream = &logFile;
bool logFiles_open = false;
bool should_log = false;

NewSoftSerial lcdSerial = NewSoftSerial(-1, 6);
NewSoftSerial gpsSerial = NewSoftSerial(4, -1);
KellyCanbus kellyCanbus = KellyCanbus(1.84);
#define TIMEZONEOFFSET -7
#define LCD_COMMAND 0xFE
#define LCD_CLEAR   0x01

#define METERSTOMILES 0.000621371192
#define MILLISPERHOUR 3600000

#define COMMA ","

#define GPSRATE 4800

/* Define Joystick connection */
#define UP     A1
#define RIGHT  A2
#define DOWN   A3
#define CLICK  A4
#define LEFT   3
#define CHIP_SELECT 9

int LED2 = 8;
int LED3 = 7;

uint16_t offsets[] = { 0, 1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768 };
int brightness = 129;

//int baseChars16Column[4] = { 0, 64, 16, 80 };
uint8_t baseChars20Column[4] = { 0, 64, 20, 84 };

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
float distance_GPS;
//float distance_RPM;
//float lastDistance_RPM;
float tripDistance_GPS;
//float tripDistance_RPM;

float milesPerKwh_GPS;
float whPerMile_GPS;
//float milesPerKwh_RPM;
//float whPerMile_RPM;

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

uint16_t file_num = 0;
#define MAX_FILES 65536
#define MAX_REPS 15

#define MAX_BUFSIZE 25
uint8_t buf_index;
char *buf_ptr;
char buffer[MAX_BUFSIZE];
const char str00[] PROGMEM = "TangoLogger";
const char str01[] PROGMEM = "Free RAM: ";
const char str02[] PROGMEM = "CAN Init...";
const char str03[] PROGMEM = "OK";
const char str04[] PROGMEM = "FAIL";
const char str05[] PROGMEM = "Log Files...";
const char str06[] PROGMEM = "MPH:";
const char str07[] PROGMEM = "C:";
const char str08[] PROGMEM = "WM:";
const char str09[] PROGMEM = "MK:";
const char str10[] PROGMEM = "Trip:";
const char str11[] PROGMEM = "NO CANBUS";
const char str12[] PROGMEM = "KellyCanbus.count: ";
const char str13[] PROGMEM = "Total iterations: ";
const char str14[] PROGMEM = "Syncing files";
const char str15[] PROGMEM = "DONE";
const char str16[] PROGMEM = "No More Files";
const char str17[] PROGMEM = "Failed opening log file: ";
const char str18[] PROGMEM = "Failed opening raw file: ";
const char str19[] PROGMEM = "LogFile: ";
const char str20[] PROGMEM = "B+:";
const char str21[] PROGMEM = "00000-LG.CSV";
PROGMEM const char *strings[] = { str00, str01, str02, str03, str04, str05, str06, str07, str08, str09, 
                                    str10, str11, str12, str13, str14, str15, str16, str17, str18, str19,  
                                    str20, str21  };

// store error strings in flash to save RAM
#define error(s) sd.errorHalt_P(PSTR(s))

uint32_t iterations = 0;
 
void setup() {
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
    analogReference(DEFAULT);
    //digitalWrite(vOutPin, HIGH );
  
    Serial.begin(115200);
    printString_P ( Serial, 0 );
    
    setSyncProvider(gpsTimeToArduinoTime);

    clear_lcd();
    lcd_move_to ( 0, 0 );
    printString_P ( lcdSerial,  0 ); // TangoLogger Init
    lcd_move_to ( 2, 0 );
    printString_P ( lcdSerial, 1 ); // Free Ram
    lcdSerial.print ( FreeRam() );
    delay ( 500 );
    lcd_move_to ( 1, 0 );
    printString_P ( lcdSerial, 2 ); // CAN INIt
    if(kellyCanbus.init()) {
        printString_P ( lcdSerial, 3 ); // OK
    } else {
        printString_P ( lcdSerial, 4 ); // FAILED
    } 
    delay ( 500 );

    initMainLogLoop();
}

void initMainLogLoop() {
    lcd_move_to ( 1, 0 );
    printString_P ( lcdSerial, 5 ); // Log Files
    init_logger();
    printString_P ( Serial, 1 ); 
    Serial.println ( FreeRam() );

    clear_lcd();
    lcd_move_to ( 0, 0 );
    printString_P ( lcdSerial, 20 ); // B+
    lcd_move_to ( 0, 15 );
    lcdSerial.print ( "!" );
    lcd_move_to ( 1, 0 );
    printString_P ( lcdSerial, 7 ); // C:
    /*
    printString_P ( lcdSerial, 6 ); // MPH:
    lcd_move_to ( 1, 0 );
    printString_P ( lcdSerial, 7 ); // C:
    lcd_move_to ( 2, 0 );
    printString_P ( lcdSerial, 8 ); // WM:
    lcd_move_to ( 2, 10 );
    printString_P ( lcdSerial, 9 ); // WM:
    lcd_move_to ( 3, 0 );
    printString_P ( lcdSerial, 10 ); // Trip:
    */
}

void loop() {
    processSerial();

    iterations++;
    currentMillis = millis();
    time_t currentTime;
    while ( gpsSerial.available() ) {
        if ( gps.encode( gpsSerial.read() ) ) {
            gps.f_get_position(&flat, &flon, &fix_age);
            fmph = gps.f_speed_mph();
            if ( fmph > 150 ) {
                fmph = 0;
            }
            //gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &fix_age);
            fcourse = gps.f_course();
            gps.get_datetime(&date, &time, &fix_age);
        }
    }

        /*
         * if we have new GPS data or it's been 1 second since the last 
         * update, perform the full update which includes reading all data,
         * updating the LCD, and writing a record to the primary logFile.
         */
    if ( currentMillis - lastFullReadMillis > 1000 ) {

        if ( ( prev_flat != flat ) || ( prev_flon != flon ) ) {
            distance_GPS = gps.distance_between ( prev_flat, prev_flon, flat, flon ) * METERSTOMILES;
            if ( distance_GPS > 10 ) {
                distance_GPS = 0;
            }
            tripDistance_GPS += distance_GPS;
            prev_flat = flat;
            prev_flon = flon;
        } else  {
            distance_GPS = 0;
        }
        kellyCanbus.fetchAllRuntimeData();
        tDiffMillis = currentMillis - lastFullReadMillis;
        //Serial.print ( "rpm: " );
        //Serial.println ( kellyCanbus.rpm, DEC );
        if ( ( should_log == false ) && ( kellyCanbus.rpm > 0 ) ) {
            should_log = true;
            Serial.println ( "shouldLog-> true" );
        }
        //distance_RPM = kellyCanbus.rpm * 80.296 (circumference in inches/rev) * 1.04530931800 (adjustment factor) / 60 (sec/min) / 1000 (ms/s) / 12 (inches/ft) / 5280 (ft/mi) / 2 (motor pole messup)
        //distance_RPM = kellyCanbus.rpm * tDiffMillis * 0.00000001103931989;
        if ( distance_GPS > 0 ) {
            whPerMile_GPS = ( kellyCanbus.wAvg * tDiffMillis / ( MILLISPERHOUR ) ) / distance_GPS;
            milesPerKwh_GPS = distance_GPS / ( kellyCanbus.wAvg * tDiffMillis / ( MILLISPERHOUR ) );
            milesPerKwh_GPS = distance_GPS / ( kellyCanbus.wAvg * tDiffMillis / ( MILLISPERHOUR ) );
        } else {
            whPerMile_GPS = 0;
            milesPerKwh_GPS = 0;
        }

        //vInReading = analogRead(vInPin);
        vOutReading = analogRead(vOutPin);
        //vIn = 5.0 * (float)vInReading / 1024.0;
        vOut = vIn / 1024.0 * (float)vOutReading;
        z2 = ( -1 * vOut * Z1 ) / ( vOut - vIn );
        c = -2 * pow(10,-5) * pow( z2, 2)  + 0.1638 * z2 - 120.28;
        //f = c * 9 / 5 + 32;

        currentTime = now();
/*
        lcd_move_to ( 0, 4 );
        lcdPrintFloat ( fmph, 4, 1 );
        lcd_move_to ( 0, 9 );
        if ( kellyCanbus.available ) {
            lcdPrintFloat ( kellyCanbus.getMPHFromRPM(), 4, 1 );
            lcdPrintFloat ( kellyCanbus.getTractionPackVoltage(), 7, 2 );
        } else {
            printString_P ( lcdSerial, 11 ); // NO CANBUS
        }
*/        

        lcd_move_to ( 0, 3 );
        lcdPrintFloat ( kellyCanbus.getTractionPackVoltage(), 6, 2 );
        lcdPrintFloat ( ( kellyCanbus.getTractionPackVoltage() / 36 ), 5, 2 );

        if ( ( should_log == true ) && ( logFiles_open == false ) ) {
            bool error = false;
            //Serial.println ( "Opening files!" );
            create_filename ( file_num );
            Serial.print ( "Trying to open: " );
            Serial.print ( buffer );
            if ( ! logFile.open ( buffer, O_WRITE | O_CREAT ) ) {
                // TODO: What now?
                error = true;
                Serial.println ( ": FAIL" );
                error ( "file.open" );
            } else {
                Serial.println ( ": OK" );
            }
            buffer[6] = 'R';
            buffer[7] = 'W';
            Serial.print ( "Trying to open: " );
            Serial.print ( buffer );
            if ( ! rawFile.open ( buffer, O_WRITE | O_CREAT ) ) {
                // TODO: What now?
                error = true;
                Serial.println ( ": FAIL" );
                error ( "file.open" );
            } else {
                Serial.println ( ": OK" );
            }

            if ( ! error ) {
                lcd_move_to ( 0, 15 );
                lcdSerial.print ( " " );
            }
            logFiles_open = true;
        }

        lcd_move_to ( 1, 2 );
        lcdPrintInt ( kellyCanbus.rawData[MOTOR_TEMP], 3, DEC );
        lcdPrintFloat ( c, 5, 1 );
        lcdPrintInt ( kellyCanbus.count, 6, DEC );
        /*
        lcd_move_to ( 1, 16 );
        lcdPrintFloat ( ( kellyCanbus.getTractionPackVoltage() / 36 ), 4, 2 );

        lcd_move_to ( 2, 3 );
        lcdPrintFloat ( whPerMile_GPS, 6, 2 );
        lcd_move_to ( 2, 13 );
        lcdPrintFloat ( milesPerKwh_GPS, 6, 2 );

        lcd_move_to ( 3, 5 );
        lcdPrintFloat ( tripDistance_GPS, 6, 2 );
        lcd_move_to ( 3, 12 );
        printIntLeadingZero ( lcdSerial, hour ( currentTime ) );
            // odd, if starting before 3,14 and printing past 3,14, weird wrapping occurs.
        lcd_move_to ( 3, 14 );
        lcdSerial.print ( ":" );
        printIntLeadingZero ( lcdSerial, minute ( currentTime ) );
        lcdSerial.print ( ":" );
        printIntLeadingZero ( lcdSerial, second ( currentTime ) );
        */

        if ( should_log ) {
            printLong ( *stream, currentMillis, DEC );
            printInt ( *stream, tDiffMillis, DEC );
            printLong ( *stream, kellyCanbus.count, DEC );
            printLong ( *stream, iterations, DEC );
            printIntLeadingZero ( *stream, year ( currentTime ) ); printIntLeadingZero ( *stream, month ( currentTime ) ); printIntLeadingZero ( *stream, day ( currentTime ) ); printString ( *stream, COMMA );
            printIntLeadingZero ( *stream, hour ( currentTime ) ); printIntLeadingZero ( *stream, minute ( currentTime ) ); printIntLeadingZero ( *stream, second ( currentTime )); printString ( *stream, COMMA );
            printFloat ( *stream, fmph, 2 );
            printFloat ( *stream, kellyCanbus.getMPHFromRPM(), 2 );
            printFloat ( *stream, kellyCanbus.getTractionPackVoltage(), 3 );
            printFloat ( *stream, flat, 5 );
            printFloat ( *stream, flon, 5 );
            printFloat ( *stream, fcourse, 2 );
            printFloat ( *stream, distance_GPS, 5 );
            printFloat ( *stream, kellyCanbus.iAvg, 4 );
            printFloat ( *stream, kellyCanbus.vAvg, 4 );
            printFloat ( *stream, kellyCanbus.wAvg, 4 );
            printFloat ( *stream, whPerMile_GPS, 5 );
            printFloat ( *stream, milesPerKwh_GPS, 5 );
            printFloat ( *stream, c, 2 );
            printInt ( *stream, vOutReading, DEC );

            for ( int i = 0; i < 22; i++ ) {
                printInt ( *stream, kellyCanbus.rawData[i], DEC );
            }
            printLine ( *stream );
        }
        lastFullReadMillis = currentMillis;
            // reset distance_GPS in case we do another round before getting another
            // set of GPS data, we don't re-calcualte wh/mi with a bogus distance_GPS.
        distance_GPS = 0;
        kellyCanbus.resetMotorInfo();

    } else {
        kellyCanbus.getCCP_A2D_BATCH_READ2();
    }
        /*
         * always write the raw current and voltage info to the raw file.
         */
    if ( should_log ) {
        rawFile.print ( currentMillis, DEC );
        rawFile.print ( COMMA );
        for ( int i = CCP_A2D_BATCH_READ2_OFFSET; i < CCP_A2D_BATCH_READ2_OFFSET + 6; i++ ) {
            rawFile.print ( kellyCanbus.rawData[i], DEC );
            rawFile.print ( COMMA );
        }
        rawFile.println();
    }

    if ( ( currentMillis - lastMillis2 ) >= 5000 ) {
        printString_P ( Serial, 12 ); // kellyCanbus.count
        Serial.println ( kellyCanbus.count, DEC );
        printString_P ( Serial, 13 ); // iterations
        Serial.println ( iterations, DEC );
        iterations = 0;
        if ( should_log ) {
            printlnString_P ( Serial, 14 );
        }
        logFile.sync();
        rawFile.sync();
        lastMillis2 = currentMillis;
    }

    if (digitalRead(CLICK) == 0){  /* Check for Click button */
        logFile.close();
        rawFile.close();
        lcd_move_to ( 1, 0 );
        printString_P ( lcdSerial, 15 ); // DONE
        printlnString_P ( Serial, 15 ); // DONE
        while ( 1 ) {
        }
    } else if (digitalRead(UP) == 0) {  /* Check for Click button */
        if ( millis() - lastClickMillis > 1000 ) {
            lastClickMillis = millis();
            brightness += 5;
            if ( brightness > 157 ) {
                brightness = 157;
            }
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
    lcdSerial.print(LCD_COMMAND,BYTE);
    lcdSerial.print(LCD_CLEAR,BYTE);
}  

void lcd_move_to ( int row, int column ) {
    int commandChar;
    commandChar = baseChars20Column[row];
    commandChar += column;
        /* set the high 7 bit to 1 per the spec */
    commandChar |= 0x80;
    lcdSerial.print(LCD_COMMAND,BYTE);
    lcdSerial.print(commandChar,BYTE);
}

void init_logger() {
    // initialize the SD card at SPI_HALF_SPEED to avoid bus errors with
    // breadboards.  use SPI_FULL_SPEED for better performance.
    // if SD chip select is not SS, the second argument to init is CS pin number
    if (! sd.init(SPI_HALF_SPEED, CHIP_SELECT)) {
        sd.initErrorHalt();
    }
    SdFile::dateTimeCallback(dateTime);

    Serial.println ( "Searching for files..." );
    file_num = offsets[15];
    uint8_t last;
    for ( int count = MAX_REPS; count >= 0; count-- ) {
        create_filename ( file_num );
        if (sd.exists(buffer)) {
            last = 1;
            file_num += offsets[count];
        } else {
            last = -1;
            file_num -= offsets[count];
        }
    }
    if ( last == 1 ) {
        file_num++;
    }
    Serial.print ( "file_num" );
    Serial.println ( file_num, DEC );

    lcd_move_to ( 1, 0 );
    printString_P ( lcdSerial, 19 );
    lcdSerial.print ( file_num, DEC );
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

void printIntLeadingZero ( Print &stream, int digits ) {
  if ( digits < 10 ) {
      stream.print ( '0' );
  }
  stream.print ( digits );
}

void printFloat ( Print &stream, float f, int places ) {
    stream.print ( f, places );
    stream.print ( COMMA );
}

void printInt ( Print &stream, int i, int type ) {
    stream.print ( i, type );
    stream.print ( COMMA );
}

void printLong ( Print &stream, long l, int type ) {
    stream.print ( l, type );
    stream.print ( COMMA );
}

void printString ( Print &stream, char *s ) {
    stream.print ( s );
    //stream.print ( COMMA );
}

void printLine ( Print &stream ) {
    stream.println();
}

void printString_P ( Print &stream, int index ) {
    strcpy_P ( buffer, (char*)pgm_read_word ( &(strings[index]) ) );
    stream.print ( buffer );
}

void printlnString_P ( Print &stream, int index ) {
    printString_P ( stream, index );
    stream.println();
}


void lcdPrintFloat ( float f, uint8_t padding, uint8_t places ) {
    uint8_t count = 0;
    if ( int ( f / 1000 ) > 0 ) {
        count = 4;
    } else if ( int ( f / 100 ) > 0 ) {
        count = 3;
    } else if ( int ( f / 10 ) > 0 ) {
        count = 2;
    } else {
        count = 1;
    }
    count += places + 1;
    if ( f < 0 ) {
        count++;
    }
    for ( uint8_t i = padding; i > count; i-- ) {
        lcdSerial.print ( " " );
    }
    lcdSerial.print ( f, places );
}

void lcdPrintInt ( long l, uint8_t padding, uint8_t type ) {
    uint8_t count = 0;
    if ( int ( l / 1000 ) > 0 ) {
        count = 4;
    } else if ( int ( l / 100 ) > 0 ) {
        count = 3;
    } else if ( int ( l / 10 ) > 0 ) {
        count = 2;
    } else {
        count = 1;
    }
    if ( l < 0 ) {
        count++;
    }
    for ( uint8_t i = padding; i > count; i-- ) {
        lcdSerial.print ( " " );
    }
    lcdSerial.print ( l, type );
}


void create_filename ( int num ) {
    strcpy_P ( buffer, (char*)pgm_read_word ( &(strings[21]) ) ); // base filename
    buffer[0] = num / 10000 + '0';
    buffer[1] = num % 10000 / 1000 + '0';
    buffer[2] = num % 1000 / 100 + '0';
    buffer[3] = num % 100 / 10 + '0';
    buffer[4] = num % 10 + '0';
}

void processSerial() {
    buf_ptr = buffer;
    if ( Serial.available() ) {
        while ( Serial.available() ) {
            *buf_ptr = Serial.read();
            if ( ( *buf_ptr == '\r' ) || ( *buf_ptr == '\n' ) ) {
                *buf_ptr = NULL;
            }
            buf_ptr++;
        }
        *buf_ptr = NULL;
        Serial.print ( "COMMAND: " );
        Serial.println ( buffer );
        if ( buffer[0] == 'L' ) {
            dir_t entry;
            Serial.println ( "START" );
            for (int i = 0; i <= file_num; i++) {
                create_filename ( i );
                for ( uint8_t j = 0; j < 2; j++ ) {
                    if ( j == 1 ) {
                        buffer[6] = 'R';
                        buffer[7] = 'W';
                    }

                    if ( readFile.open ( buffer ) ) {
                        readFile.dirEntry ( &entry );
                        Serial.print ( buffer );
                        Serial.print ( COMMA );
                        Serial.print ( entry.lastWriteDate );
                        Serial.print ( COMMA );
                        Serial.print ( entry.lastWriteTime );
                        Serial.print ( COMMA );
                        Serial.print ( entry.fileSize );
                        Serial.println();
                        readFile.close();
                    }
                }
            }
            Serial.println ( "END" );
        } 
    }
}
