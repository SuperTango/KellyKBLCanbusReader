#include <SdFat.h>
#include <SdFatUtil.h>
#include <TinyGPS.h>
#include <NewSoftSerial.h>
#include <KellyCanbus.h>
#include <Time.h>
#include <avr/pgmspace.h>

#define STATE_GATHERDATA 1
#define STATE_TRANSFERFILES 2

#undef DEBUG_WHLOGS
#undef DEBUG_GPS

#define LCD_TYPE_20X4 0
#define LCD_TYPE_16X2 1
uint8_t lcd_type = LCD_TYPE_20X4;

SdFat sd;
SdFile logFile;
//SdFile rawFile;
SdFile nmeaFile;
SdFile readFile;

//Print *stream = &Serial;
Print *stream = &logFile;
bool logFiles_open = false;
bool should_log = false;

int lcd_clear_count = 0;
NewSoftSerial lcdSerial = NewSoftSerial(-1, 6);
NewSoftSerial gpsSerial = NewSoftSerial(4, -1);
KellyCanbus kellyCanbus = KellyCanbus(1.84);
unsigned short failed_cs;
unsigned short failed_cs_last;
unsigned short failed_cs_diff;
#define TIMEZONEOFFSET -8
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

/*
 * stuff for time keeping
 */
int cur_year;
tmElements_t tm;
time_t cur_time;
time_t last_gps_time;

//uint8_t baseChars16Column[4] = { 0, 64, 16, 80 };
uint8_t baseChars20Column[4] = { 0, 64, 20, 84 };

unsigned long currentMillis;
unsigned long lastSaveMillis = 0;
unsigned long lastClickMillis = 0;
unsigned long tDiffMillis;

/*
 * Vars needed by tinyGPS
 */
TinyGPS gps;
long lat, lon;
unsigned long fix_age, course;
unsigned long chars;
long altitude;
float flat, flon, speed_GPS, fcourse;
float prev_flat, prev_flon;
float distance_GPS;
float distance_RPM;
float speed_RPM;
//float lastDistance_RPM;
float tripDistance_GPS;
float tripDistance_RPM;

float batteryWh;
float batteryWhTotal = 0;
float milesPerKwh_GPS;
float whPerMile_GPS;
float milesPerKwh_RPM = 0;
float whPerMile_RPM = 0;
float milesPerKwh_Trip;
float whPerMile_Trip;

#define MOTOR_THERMISTOR_PIN A0
#define MOTOR_5V_PIN  A1
//int motor5VReading;
int motorThermistorReading;
//float motor5VActual;
#define motor5VActual 5.0
float vOut;
#define Z1 1000.0
float z2;
float c;

#define BATTERY_CURRENT_SENSOR_PIN A5
long  batteryCurrentReadingTotal = 0;
float batteryCurrentReadingAvg;
float batteryCurrentReadingSingle;
float batteryCurrentAvg;
float batteryCurrentSingle;

float batteryVoltage;
float watts_sensor = 0;

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
const char str06[] PROGMEM = "S:";
const char str07[] PROGMEM = "C:";
const char str08[] PROGMEM = "Wm:";
const char str09[] PROGMEM = "mK:";
const char str10[] PROGMEM = "D:";
const char str11[] PROGMEM = "NO CANBUS";
const char str12[] PROGMEM = "Failed Checksums:  ";
const char str13[] PROGMEM = "Wh:";
const char str14[] PROGMEM = "Syncing files";
const char str15[] PROGMEM = "DONE";
const char str16[] PROGMEM = "No More Files";
const char str17[] PROGMEM = "Failed opening log file: ";
const char str18[] PROGMEM = "Failed opening raw file: ";
const char str19[] PROGMEM = "LogFile: ";
const char str20[] PROGMEM = "V:";
const char str21[] PROGMEM = "00000-LG.CSV";
const char str22[] PROGMEM = "Does not exist";
const char str23[] PROGMEM = "START ";
const char str24[] PROGMEM = "END";
const char str25[] PROGMEM = "I:";
const char str26[] PROGMEM = "READY";
const char str27[] PROGMEM = "#LOGFMT 5";
PROGMEM const char *strings[] = { str00, str01, str02, str03, str04, str05, str06, str07, str08, str09, 
                                    str10, str11, str12, str13, str14, str15, str16, str17, str18, str19,  
                                    str20, str21, str22, str23, str24, str25, str26, str27 };

// store error strings in flash to save RAM
#define error(s) sd.errorHalt_P(PSTR(s))

bool got_data = false;
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
    //digitalWrite(MOTOR_THERMISTOR_PIN, HIGH );
  
    Serial.begin(115200);
    printString_P ( Serial, 0 );
    
    setSyncProvider(gpsTimeToArduinoTime);

    batteryWh = 0;
    batteryWhTotal = 0;

    lcd_clear();
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

    lcd_move_to ( 1, 0 );
    printString_P ( lcdSerial, 5 ); // Log Files
    init_logger();
    printString_P ( Serial, 1 ); 
    Serial.println ( FreeRam() );

    initLCD();
    printlnString_P ( Serial, 26 ); 
}

void initLCD() { 
    lcd_clear();
    if ( lcd_type == LCD_TYPE_16X2 ) {
        lcd_move_to ( 0, 0 );
        printString_P ( lcdSerial, 20 ); // V:
        lcd_move_to ( 1, 0 );
        printString_P ( lcdSerial, 7 ); // C:
        lcd_move_to ( 1, 15 );
        if ( ! should_log ) {
            lcdSerial.print ( "!" );
        }

    } else {
        lcd_move_to ( 0, 0 );
        printString_P ( lcdSerial, 20 ); // V:
        lcd_move_to ( 0, 13 );
        printString_P ( lcdSerial, 25 ); // I:
        
        lcd_move_to ( 1, 0 );
        printString_P ( lcdSerial, 8 ); // wm
        lcd_move_to ( 1, 19 );
        if ( ! should_log ) {
            lcdSerial.print ( "!" );
        }

        lcd_move_to ( 2, 0 );
        printString_P ( lcdSerial, 9 ); // mk:
        lcd_move_to ( 2, 15 );
        printString_P ( lcdSerial, 7 ); // C:

        lcd_move_to ( 3, 0 );
        printString_P ( lcdSerial, 10 ); // D:
        lcd_move_to ( 3, 7 );
        printString_P ( lcdSerial, 13 ); // D:
    }
}

float convertBatteryCurrent ( float batteryCurrentReading ) {
        // return batteryCurrentReading * -0.9638554 + 799.0361446 + 2.5; // For CSLA2DK Backwards
        return batteryCurrentReading * 0.9638554 - 799.0361446 + 2.5; // For CSLA2DK Forwards
        // return batteryCurrentReading * -05421687 + 449.4578313 + 2.5; // For CSL1EJ Backwards
        // return batteryCurrentReading * 0.9638554 - 449.4578313 + 2.5; // For CSLA1EJ Forwards
}

void loop() {
    processSerial();

    iterations++;
    currentMillis = millis();
    char gpsByte;
    got_data = false;
    batteryCurrentReadingTotal += analogRead ( BATTERY_CURRENT_SENSOR_PIN );
    while ( gpsSerial.available() ) {
        gpsByte = gpsSerial.read();
        if ( should_log ) {
            nmeaFile.print ( gpsByte );
#ifdef DEBUG_GPS
            Serial.print ( gpsByte );
#endif
        }
        if ( gps.encode( gpsByte ) ) {
            //Serial.println ( "GOT GOOD DATA" );
            gps.f_get_position(&flat, &flon, &fix_age);
            speed_GPS = gps.f_speed_mph();
            if ( speed_GPS > 150 ) {
                speed_GPS = 0;
            }
            gps.crack_datetime( &cur_year, &tm.Month, &tm.Day, &tm.Hour, &tm.Minute, &tm.Second, NULL, &fix_age );
            tm.Year = cur_year - 1970;
            fcourse = gps.f_course();
            altitude = gps.altitude();
            cur_time = makeTime ( tm );
            if ( cur_time != last_gps_time ) {
                got_data = true;
                tDiffMillis = ( cur_time - last_gps_time ) * 1000;
                last_gps_time = cur_time;
                    // problem exists when acquiring time signal where tDiffMillis is way off.
                if ( tDiffMillis < 0 ) {
                    got_data = false;
                }
            }
        }
    }

        /*
         * if we have new GPS data or it's been 1 second since the last 
         * update, perform the full update which includes reading all data,
         * updating the LCD, and writing a record to the primary logFile.
         */
    if ( got_data ) {
        lcd_clear_count++;
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

        if ( ( should_log == false ) && ( kellyCanbus.rpm > 0 ) ) {
            should_log = true;
        }

        batteryCurrentReadingSingle = analogRead ( BATTERY_CURRENT_SENSOR_PIN );
        batteryCurrentReadingAvg = batteryCurrentReadingTotal / iterations;
        batteryCurrentAvg = convertBatteryCurrent ( batteryCurrentReadingAvg );
        batteryCurrentSingle = convertBatteryCurrent ( batteryCurrentReadingSingle );

        batteryVoltage = kellyCanbus.getTractionPackVoltage();
        watts_sensor = batteryCurrentAvg * batteryVoltage;
        batteryWh = watts_sensor * tDiffMillis / MILLISPERHOUR;
        if ( batteryWh > 10 ) {
            batteryWh = 0;
        }
        batteryWhTotal += batteryWh;
#ifdef DEBUG_WHLOGS
        Serial.print ( ", batteryWh: " );
        Serial.print ( batteryWh );
        Serial.print ( ", batteryWhTotal: " );
        Serial.print ( batteryWhTotal );
#endif


            /*
             * The conversion of RPMs * diff_millis to a linear distance was done by taking a ride
             * of distance 5.69738 miles, and summing up the wheel revolutions (rpm * diff_millis)
             * which equals 9339.3333 revolutions.  
             *
             * Then taking 9339.3333 revs / 5.69738 miles = 1639.233004 revs/mi
             *
             * So distance_RPM = X rev   Y ms   1min    1sec     1 mile
             *                     --- *      * --    * ----- * ---------------
             *                     min          60 sec  1000ms   1639.233004 revs
             *
             * Reducing, distance_RPM = X rev/min * Y ms / 1.01673567 * 10E-8.
             *
             * Since that's too small a number for floating point on arduino to be precise, we use a
             * constant of 1.01673567 and then divide by 1E8 (100000000) later.
             *
             * The speed_RPM should also derive from this equation, but for some reason, it doesn't,
             * so i calculated a constant by taking the average GPS speed when the RPM number was
             * 1942 (chosen since it was a high speed and there were a few different GPS readings for
             * that RPM), then divided.  The constant there is 0.037224511.
             */
        distance_RPM = kellyCanbus.rpm * tDiffMillis * 1.01673567; // remember to divide by 100000000 later!
        tripDistance_RPM += ( distance_RPM / 100000000 );
        speed_RPM = kellyCanbus.rpm * 0.037224511; 
        if ( distance_GPS > 0 ) {
            whPerMile_GPS = batteryWh / distance_GPS;
            milesPerKwh_GPS = distance_GPS / batteryWh * 1000;
        } else {
            whPerMile_GPS = 0;
            milesPerKwh_GPS = 0;
        }
        if ( distance_RPM > 0 ) {
            whPerMile_RPM = batteryWh / ( distance_RPM / 100000000 );
            milesPerKwh_RPM = distance_RPM / 100000 / batteryWh;
            if ( whPerMile_RPM >= 250 ) {
                whPerMile_RPM = 250;
            }
            if ( milesPerKwh_RPM >= 99 ) {
                milesPerKwh_RPM = 99;
            }
            whPerMile_Trip = batteryWhTotal / tripDistance_RPM;
            milesPerKwh_Trip = tripDistance_RPM / batteryWhTotal * 1000;
#ifdef DEBUG_WHLOGS
            Serial.print ( ", mk: " );
            Serial.print ( milesPerKwh_RPM );
            Serial.print ( ", trip: " );
            Serial.print ( milesPerKwh_Trip );

            Serial.print ( ", wm: " );
            Serial.print ( whPerMile_RPM );
            Serial.print ( ", trip: " );
            Serial.print ( whPerMile_Trip );
#endif
        } else {
            whPerMile_RPM = 0;
            milesPerKwh_RPM = 0;
        }
#ifdef DEBUG_WHLOGS
            Serial.println();
#endif

        //motor5VReading = analogRead(MOTOR_5V_PIN);
        motorThermistorReading = analogRead(MOTOR_THERMISTOR_PIN);
        //motor5VActual = 5.0 * (float)motor5VReading / 1024.0;
        vOut = motor5VActual / 1024.0 * (float)motorThermistorReading;
        z2 = ( -1 * vOut * Z1 ) / ( vOut - motor5VActual );
        c = -2 * pow(10,-5) * pow( z2, 2)  + 0.1638 * z2 - 120.28;
        //f = c * 9 / 5 + 32;

/*
        lcd_move_to ( 0, 4 );
        lcdPrintFloat ( speed_GPS, 4, 1 );
        lcd_move_to ( 0, 9 );
        if ( kellyCanbus.available ) {
            lcdPrintFloat ( kellyCanbus.getMPHFromRPM(), 4, 1 );
            lcdPrintFloat ( batteryVoltage, 7, 2 );
        } else {
            printString_P ( lcdSerial, 11 ); // NO CANBUS
        }
*/        


        if ( ( should_log == true ) && ( logFiles_open == false ) ) {
            bool error = false;
            create_filename ( file_num );
            if ( ! logFile.open ( buffer, O_WRITE | O_CREAT ) ) {
                error = true;
                error ( "file.open1" );
            }
            /*
            buffer[6] = 'R';
            buffer[7] = 'W';
            if ( ! rawFile.open ( buffer, O_WRITE | O_CREAT ) ) {
                error = true;
                error ( "file.open2" );
            }
            */
            buffer[6] = 'N';
            buffer[7] = 'M';
            buffer[9] = 'g';
            buffer[10] = 'p';
            buffer[11] = 's';
            //buffer[13] = NULL;
            if ( ! nmeaFile.open ( buffer, O_WRITE | O_CREAT ) ) {
                error = true;
                error ( "file.open3" );
            }

            if ( ! error ) {
                if ( lcd_type == LCD_TYPE_16X2 ) {
                    lcd_move_to ( 1, 15 );
                } else {
                    lcd_move_to ( 2, 19 );
                }
                lcdSerial.print ( " " );
            }
            logFiles_open = true;
            printlnString_P ( *stream, 27 ); // Output Format type
        }

        if ( lcd_clear_count > 10 ) {
            initLCD();
            lcd_clear_count = 0;
        }

        lcd_move_to ( 0, 2 );
        lcdPrintFloat ( batteryVoltage, 5, 1 );
        lcdPrintFloat ( ( batteryVoltage / 36 ), 5, 2 );

        if ( lcd_type == LCD_TYPE_20X4 ) {
            lcd_move_to ( 0, 15 );
            if ( batteryCurrentAvg <= -100 ) {
                batteryCurrentAvg = 0;
            } else if ( batteryCurrentAvg < 0 ) {
                lcdPrintFloat ( batteryCurrentAvg, 4, 1 );
            } else {
                lcdPrintFloat ( batteryCurrentAvg, 5, 1 );
            }
        }

        lcd_move_to ( 1, 3 );
        lcdPrintFloat ( whPerMile_RPM, 6, 2 );
        lcdPrintFloat ( whPerMile_Trip, 7, 2 );
        if ( should_log ) {
            lcd_move_to ( 1, 19 );
            lcdSerial.print ( " " );
        }

        lcd_move_to ( 2, 3 );
        lcdPrintFloat ( milesPerKwh_RPM, 5, 2 );
        lcdPrintFloat ( milesPerKwh_Trip, 6, 2 );
        lcd_move_to ( 2, 17 );
        lcdPrintInt ( kellyCanbus.rawData[MOTOR_TEMP], 3, DEC );

        lcd_move_to ( 3, 2 );
        lcdPrintFloat ( tripDistance_RPM, 4, 1 );
        lcd_move_to ( 3, 10 );
        lcdPrintInt ( int ( batteryWhTotal ), 4, DEC );
        lcd_move_to ( 3, 15 );
        int tzHour = tm.Hour + TIMEZONEOFFSET;
        if ( tzHour < 0 ) {
            tzHour += 24;
        } else if ( tzHour >= 24 ) {
            tzHour -= 24;
        }
        printIntLeadingZero ( lcdSerial, tzHour );
        lcdSerial.print ( ":" );
        printIntLeadingZero ( lcdSerial, tm.Minute );

        if ( should_log ) {
            printLong ( *stream, currentMillis, DEC );
            printInt ( *stream, tDiffMillis, DEC );
            printLong ( *stream, kellyCanbus.count, DEC );
            printLong ( *stream, iterations, DEC );
            printIntLeadingZero ( *stream, cur_year ); printIntLeadingZero ( *stream, tm.Month ); printIntLeadingZero ( *stream, tm.Day ); printString ( *stream, COMMA );
            printIntLeadingZero ( *stream, tm.Hour ); printIntLeadingZero ( *stream, tm.Minute ); printIntLeadingZero ( *stream, tm.Second ); printString ( *stream, COMMA );
            printInt ( *stream, fix_age, DEC );
            printFloat ( *stream, speed_GPS, 2 );
            printFloat ( *stream, speed_RPM, 2 );
            printFloat ( *stream, flat, 5 );
            printFloat ( *stream, flon, 5 );
            printFloat ( *stream, fcourse, 2 );
            printInt ( *stream, altitude, DEC );
            printInt ( *stream, failed_cs_diff, DEC );
            printFloat ( *stream, distance_GPS, 5 );
            printFloat ( *stream, distance_RPM, 5 );
            printFloat ( *stream, batteryVoltage, 3 );
            printFloat ( *stream, batteryCurrentReadingTotal, DEC );
            printFloat ( *stream, batteryCurrentAvg, 5 );
            printFloat ( *stream, batteryCurrentReadingSingle, DEC );
            printFloat ( *stream, batteryCurrentSingle, 5 );
            printFloat ( *stream, batteryWh, 5 );
            printFloat ( *stream, batteryWhTotal, 5 );
            printFloat ( *stream, kellyCanbus.iAvg, 4 );
            printFloat ( *stream, kellyCanbus.vAvg, 4 );
            printFloat ( *stream, kellyCanbus.wAvg, 4 );
            printFloat ( *stream, whPerMile_GPS, 5 );
            printFloat ( *stream, whPerMile_RPM, 5 );
            printFloat ( *stream, whPerMile_Trip, 5 );
            printFloat ( *stream, milesPerKwh_GPS, 5 );
            printFloat ( *stream, milesPerKwh_RPM, 5 );
            printFloat ( *stream, milesPerKwh_Trip, 5 );
            printFloat ( *stream, c, 2 );
            printInt ( *stream, motorThermistorReading, DEC );

            for ( int i = 0; i < 22; i++ ) {
                printInt ( *stream, kellyCanbus.rawData[i], DEC );
            }
            printLine ( *stream );
        }
            // reset distance_GPS in case we do another round before getting another
            // set of GPS data, we don't re-calcualte wh/mi with a bogus distance_GPS.
        distance_GPS = 0;
        kellyCanbus.resetMotorInfo();
        iterations = 0;
        batteryCurrentReadingTotal = 0;

    //} else {
        //kellyCanbus.getCCP_A2D_BATCH_READ2();
    }
        /*
         * always write the raw current and voltage info to the raw file.
         */
/*
    if ( should_log ) {
        rawFile.print ( currentMillis, DEC );
        rawFile.print ( COMMA );
        for ( int i = CCP_A2D_BATCH_READ2_OFFSET; i < CCP_A2D_BATCH_READ2_OFFSET + 6; i++ ) {
            rawFile.print ( kellyCanbus.rawData[i], DEC );
            rawFile.print ( COMMA );
        }
        rawFile.println();
    }
*/

    if ( ( currentMillis - lastSaveMillis ) >= 5000 ) {
        printString_P ( Serial, 12 ); // Failed Checksums
        gps.stats ( NULL, NULL, &failed_cs );
        failed_cs_diff = failed_cs - failed_cs_last;
        failed_cs_last = failed_cs;
        Serial.println ( failed_cs_diff, DEC );
        if ( should_log ) {
            printlnString_P ( Serial, 14 ); // syncing files.
        }
        logFile.sync();
        //rawFile.sync();
        nmeaFile.sync();
        lastSaveMillis = currentMillis;
    }

    if (digitalRead(CLICK) == 0){  /* Check for Click button */
        logFile.close();
        //rawFile.close();
        nmeaFile.close();
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
    } else if (digitalRead(RIGHT) == 0) {  /* Check for Click button */
        if ( millis() - lastClickMillis > 1000 ) {
            lcd_type = ( lcd_type == LCD_TYPE_20X4 ) ? LCD_TYPE_16X2 : LCD_TYPE_20X4;
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

void lcd_clear(void)
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
    SdFile::dateTimeCallback(tangoDateTimeCallback);

    //Serial.println ( "Searching for files..." );
    file_num = offsets[MAX_REPS + 1];
    uint8_t last;
    for ( int count = MAX_REPS; count >= 0; count-- ) {
        create_filename ( file_num );
        //Serial.print ( "count: " );
        //Serial.print ( count, DEC );
        //Serial.print ( ", file_num: " );
        //Serial.print ( file_num, DEC );
        //Serial.print ( ", file: " );
        Serial.print ( buffer );
        if (sd.exists(buffer)) {
            last = 1;
            file_num += offsets[count];
            //Serial.print ( ", exists adding: " );
        } else {
            last = -1;
            file_num -= offsets[count];
            //Serial.print ( ", does not exist, subtracting: " );
        }
        //Serial.print ( offsets[count] , DEC );
        //Serial.println();
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

void tangoDateTimeCallback(uint16_t* date, uint16_t* time) {
    // return date using FAT_DATE macro to format fields
    *date = FAT_DATE ( cur_year, tm.Month, tm.Day );

    // return time using FAT_TIME macro to format fields
    *time = FAT_TIME ( tm.Hour, tm.Minute, tm.Second );
}

time_t gpsTimeToArduinoTime(){
  return cur_time + (TIMEZONEOFFSET * SECS_PER_HOUR);
}

void printMillis ( Print &stream, int digits ) {
    if ( digits < 100 ) {
        stream.print ( '0' );
    }
    printIntLeadingZero ( stream, digits );
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


void create_filename ( uint16_t num ) {
    strcpy_P ( buffer, (char*)pgm_read_word ( &(strings[21]) ) ); // base filename
    buffer[0] = num / 10000 + '0';
    buffer[1] = num % 10000 / 1000 + '0';
    buffer[2] = num % 1000 / 100 + '0';
    buffer[3] = num % 100 / 10 + '0';
    buffer[4] = num % 10 + '0';
}

void processSerial() {
    buf_ptr = buffer;
    if ( Serial.available() > 0 ) {
        while ( Serial.available() ) {
            *buf_ptr = Serial.read();
            Serial.print ( "read: " );
            Serial.println ( *buf_ptr );
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
            printString_P ( Serial, 23 ); // START
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
                        SdFile::printFatDate ( entry.lastWriteDate );
                        Serial.print ( COMMA );
                        SdFile::printFatTime ( entry.lastWriteTime );
                        Serial.print ( COMMA );
                        Serial.print ( entry.fileSize );
                        Serial.println();
                        readFile.close();
                    }
                }
            }
            printString_P ( Serial, 24 ); // END
        } else if ( buffer[0] == 'G' ) {
            buf_ptr = buffer + 2;
            Serial.print ( "buf_ptr: '" );
            Serial.print ( buf_ptr );
            Serial.println ( "'" );
            Serial.print ( "Buffer: " );
            Serial.println ( buffer );
            if ( readFile.open ( buf_ptr ) ) {
                dir_t entry;
                readFile.dirEntry ( &entry );
                Serial.print ( "file:  " );
                Serial.println ( buf_ptr );
                Serial.print ( "file size: " );
                Serial.println ( entry.fileSize, DEC );
                printString_P ( Serial, 23 ); // START
                int16_t n; 
                while ( ( n = readFile.read ( buffer, MAX_BUFSIZE ) ) > 0 ) { 
                    for ( uint8_t i = 0; i < n; i++ ) {
                        Serial.print ( buffer[i] );
                    }
                }
                printString_P ( Serial, 24 ); // END
                readFile.close();
            } else {
                printlnString_P ( Serial, 4 ); // FAIL
            }
        } else if ( buffer[0] == 'D' ) {
            buf_ptr = buffer + 2;
            if ( readFile.open ( buf_ptr ) ) {
                readFile.remove();
                printString_P ( Serial, 3 ); // OK
            } else {
                printlnString_P ( Serial, 4 ); // FAIL
            }
        }
    }
}
