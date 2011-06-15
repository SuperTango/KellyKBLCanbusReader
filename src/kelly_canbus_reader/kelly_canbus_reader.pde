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

NewSoftSerial lcd = NewSoftSerial(3, 6);
NewSoftSerial gps = NewSoftSerial(4, 5);
#define COMMAND 0xFE
#define CLEAR   0x01
#define LCD_SIZE 16
#define LINE0   0x80
#define LINE1   0xC0
#define LINE2   0x94
#define LINE3   0xd4

#define KNOTSTOMPH 1.15077945


#define GPSRATE 4800
//#define GPSRATE 38400

// GPS parser for 406a
#define BUFFSIZ 90 // plenty big

/* Define Joystick connection */
#define UP     A1
#define RIGHT  A2
#define DOWN   A3
#define CLICK  A4
#define LEFT   3
#define CHIP_SELECT 9

char buffer[BUFFSIZ];  //Data will be temporarily stored to this buffer before being written to the file
char lat_str[14];
char lon_str[14];

int LED2 = 8;
int LED3 = 7;

int iterations;

int brightness = 129;

//int baseChars16Column[4] = { 0, 64, 16, 80 };
int baseChars20Column[4] = { 0, 64, 20, 84 };

unsigned long lastMillis = 0;
unsigned long lastMillis2 = 0;
unsigned long lastClickMillis = 0;


// store error strings in flash to save RAM
#define error(s) sd.errorHalt_P(PSTR(s))

char *parseptr;
char buffidx;
uint8_t hour, minute, second, year, month, date;
uint32_t latitude, longitude;
uint8_t groundspeed, trackangle;
char latdir, longdir;
char status;
KellyCanbus kellyCanbus = KellyCanbus(1.84);
 
void setup() {
    uint16_t ret;
    Serial.begin(GPSRATE);
    lcd.begin(9600);              /* Setup serial LCD and clear the screen */
    gps.begin(GPSRATE);
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

    clear_lcd();
    move_to ( 0, 0 );
    lcd.print ( "MotoLogger Init" );
    move_to ( 1,0 );
    if(kellyCanbus.init()) {
        lcd.print("CAN Init ok");
    } else {
        lcd.print("CAN Init failed");
    } 
    delay ( 500 );
    move_to ( 1, 0 );

    Serial.print ( "FreeRam: " ); Serial.println ( FreeRam() );
    Serial.print ( "going to init_logger" );
    init_logger();
    lcd.print ("logger OK         " );
    Serial.print ( "back from init_logger" );
    Serial.print ( "FreeRam: " ); Serial.println ( FreeRam() );

    iterations = 0;
}
 
void loop() {
    bool gotGPSData = false;
    iterations++;

    //memset ( buffer, 1, 10  );
    kellyCanbus.fetchRuntimeData();
    gotGPSData = read_gps();
    if ( ( millis() - lastMillis ) >= 1000 ) {
        lastMillis = millis();
        move_to ( 0, 0 );
        lcd.print ( groundspeed * KNOTSTOMPH, 2 );
        lcd.print ( " " );
        lcd.print ( kellyCanbus.getMPHFromRPM(), 2 );
        lcd.print ( " " );
        lcd.print ( (float)iterations / (float)millis() * (float)1000, 1 );
        move_to ( 1, 0 );
        lcd.print ( "B+: " );
        lcd.print ( kellyCanbus.getTractionPackVoltage(), 3 );
    }
    
    file.print ( millis(), DEC );
    file.print ( "," );
    file.print ( year, DEC );
    file.print ( month, DEC );
    file.print ( date, DEC );
    file.print ( "_" );
    file.print ( hour, DEC );
    file.print ( minute, DEC );
    file.print ( second, DEC );
    file.print ( "," );
    file.print ( groundspeed * KNOTSTOMPH, 2 );
    file.print ( "," );
    file.print ( kellyCanbus.getMPHFromRPM(), 2 );
    file.print ( "," );
    file.print ( kellyCanbus.getTractionPackVoltage(), 3 );
    file.print ( "," );
    file.print ( lat_str );
    file.print ( "," );
    file.print ( lon_str );
    file.print ( "," );
    file.print ( groundspeed, DEC );
    file.print ( "," );
    file.print ( trackangle, DEC );
    file.print ( "," );
    for ( int i = 0; i < 22; i++ ) {
        file.print ( kellyCanbus.rawData[i], DEC );
        file.print ( "," );
    }
    file.println();
    if ( ( millis() - lastMillis2 ) >= 5000 ) {
        Serial.println ( "syncing..." );
        file.sync();
        lastMillis2 = millis();
        //while ( 1 );
    }

    if (digitalRead(CLICK) == 0){  /* Check for Click button */
        file.println ( "Closing" );
        file.close();
        Serial.println("Done");
        move_to ( 2, 0 );
        lcd.print("DONE");
        while ( 1 ) {
        }
    } else if (digitalRead(UP) == 0) {  /* Check for Click button */
        if ( millis() - lastClickMillis > 1000 ) {
            lastClickMillis = millis();
            brightness += 5;
            if ( brightness > 157 ) {
                brightness = 157;
            }
            Serial.print ( "setting brightness to: " );
            Serial.println ( brightness, DEC );
            lcd.print ( 0x7C, BYTE );
            lcd.print ( brightness, BYTE );
        }
    } else if (digitalRead(DOWN) == 0) {  /* Check for Click button */
        if ( millis() - lastClickMillis > 1000 ) {
            lastClickMillis = millis();
            brightness -= 5;
            if ( brightness < 128 ) {
                brightness = 128;
            }
            Serial.print ( "setting brightness to: " );
            Serial.println ( brightness, DEC );
            lcd.print ( 0x7C, BYTE );
            lcd.print ( brightness, BYTE );
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


bool read_gps(void) {
    uint32_t tmp;
    unsigned char i;
    unsigned char exit = 0;

    while( exit == 0) { 
        
            /*
             * bail out if the non-blocking readline doesn't have any data
             */
        if ( ! readline() ) {
            return false;
        }
     
              // check if $GPRMC (global positioning fixed data)
        if (strncmp(buffer, "$GPRMC",6) == 0) {
            digitalWrite(LED2, HIGH);
            
            // hhmmss time data
            parseptr = buffer+7;
            tmp = parsedecimal(parseptr); 
            hour = tmp / 10000;
            minute = (tmp / 100) % 100;
            second = tmp % 100;
            
            parseptr = strchr(parseptr, ',') + 1;
            status = parseptr[0];
            parseptr += 2;
              
            for(i=0;i<11;i++)
            {
                lat_str[i] = parseptr[i];
            }
            lat_str[12] = 0;
            // Serial.println(" ");
            // Serial.println(lat_str);
           
            // grab latitude & long data
            latitude = parsedecimal(parseptr);
            if (latitude != 0) {
                latitude *= 10000;
                parseptr = strchr(parseptr, '.')+1;
                latitude += parsedecimal(parseptr);
            }
            parseptr = strchr(parseptr, ',') + 1;
            // read latitude N/S data
            if (parseptr[0] != ',') {
              latdir = parseptr[0];
            }
            
            // longitude
            parseptr = strchr(parseptr, ',')+1;
          
            for(i=0;i<12;i++)
            {
              lon_str[i] = parseptr[i];
            }
            lon_str[13] = 0;
            
            //Serial.println(lon_str);
       
            longitude = parsedecimal(parseptr);
            if (longitude != 0) {
                longitude *= 10000;
                parseptr = strchr(parseptr, '.')+1;
                longitude += parsedecimal(parseptr);
            }
            parseptr = strchr(parseptr, ',')+1;
            // read longitude E/W data
            if (parseptr[0] != ',') {
                longdir = parseptr[0];
            }
        
            // groundspeed
            parseptr = strchr(parseptr, ',')+1;
            groundspeed = parsedecimal(parseptr);
        
            // track angle
            parseptr = strchr(parseptr, ',')+1;
            trackangle = parsedecimal(parseptr);
        
            // date
            parseptr = strchr(parseptr, ',')+1;
            tmp = parsedecimal(parseptr); 
            date = tmp / 10000;
            month = (tmp / 100) % 100;
            year = tmp % 100;
            
            digitalWrite(LED2, LOW);
            exit = 1;
        }
    }
    return true;
}

bool readline(void) {
  char c;
  int available;
  
  buffidx = 0; // start at begninning
  if ( ! gps.available() ) {
      return false;
  }
  while (1) {
      c=gps.read();
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

uint32_t parsedecimal(char *str) {
  uint32_t d = 0;
  
  while (str[0] != 0) {
   if ((str[0] > '9') || (str[0] < '0'))
     return d;
   d *= 10;
   d += str[0] - '0';
   str++;
  }
  return d;
}

void clear_lcd(void)
{
    lcd.print(COMMAND,BYTE);
    lcd.print(CLEAR,BYTE);
}  

void move_to ( int row, int column ) {
    int commandChar;
    commandChar = baseChars20Column[row];
    commandChar += column;
        /* set the high 7 bit to 1 per the spec */
    commandChar |= 0x80;
    lcd.print(COMMAND,BYTE);
    lcd.print(commandChar,BYTE);
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

        Serial.println ( "Creating filename" );
    // create a new file
    char name[] = "LG_00000.TXT";
    for (int i = 0; i <= 65530; i++) {
        if ( i == 65530 ) {
            clear_lcd();
            move_to ( 0,0 );
            Serial.println ( "No more filenames!" );
            lcd.print ( "No more filenames!" );
            while ( 1 ) {
            }
        }
        name[3] = i/10000 + '0';
        name[4] = i%10000 / 1000 + '0';
        name[5] = i%1000 / 100 + '0';
        name[6] = i%100 / 10 + '0';
        name[7] = i%10 + '0';
        Serial.print ( "checking" );
        Serial.println ( name );
        if (sd.exists(name)) {
            continue;
        }
        if ( file.open ( name, O_WRITE | O_CREAT ) ) {
            break;
        } else {
            Serial.print ( "Failed opening file: " );
            Serial.print ( name );
        }
    }
    Serial.print ( "Filename2: " );
    Serial.println ( name );
    //file.open ( name );
    Serial.println ( "PRE_A" );
    Serial.println ( "A" );
    SdFile::dateTimeCallback(dateTime);
    if (file.isOpen() ) {
        Serial.println ( "Open succeeded" );
        file.write ( "Starting!\n" );
        file.sync();
    } else {
        Serial.println ( "B" );
        error ("file.open");
    }
}

void dateTime(uint16_t* date, uint16_t* time) {

  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE((uint16_t)year, (uint16_t)month, (uint16_t)date );

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME((uint16_t)hour, (uint16_t)minute, (uint16_t)second );
}

