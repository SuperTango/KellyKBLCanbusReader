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


Sd2Card card;
SdVolume volume;
SdFile root;
SdFile file;

NewSoftSerial sLCD =  NewSoftSerial(3, 6); /* Serial LCD is connected on pin 14 (Analog input 0) */
#define COMMAND 0xFE
#define CLEAR   0x01
#define LCD_SIZE 16
#define LINE0   0x80
#define LINE1   0xC0
#define LINE2   0x94
#define LINE3   0xd4


/* Define Joystick connection */
#define UP     A1
#define RIGHT  A2
#define DOWN   A3
#define CLICK  A4
#define LEFT   3

  
char buffer[512];  //Data will be temporarily stored to this buffer before being written to the file
char tempbuf[15];
char lat_str[14];
char lon_str[14];

int read_size=0;   //Used as an indicator for how many characters are read from the file

int D10 = 10;

int LED2 = 8;
int LED3 = 7;

tCAN canbusMessage;
int iterations;
bool gotMessage;

int baseChars16Column[4] = { 0, 64, 16, 80 };
int baseChars20Column[4] = { 0, 64, 20, 84 };

unsigned long lastMillis = 0;

// store error strings in flash to save RAM
#define error(s) error_P(PSTR(s))

void error_P(const char* str) {
  PgmPrint("error: ");
  SerialPrintln_P(str);
  
  clear_lcd();
  sLCD.print("SD error");
  
  if (card.errorCode()) {
    PgmPrint("SD error: ");
    Serial.print(card.errorCode(), HEX);
    
    Serial.print(',');
    Serial.println(card.errorData(), HEX);
   
  }
  while(1);
}


NewSoftSerial mySerial =  NewSoftSerial(4, 5);

#define COMMAND 0xFE
//#define powerpin 4

#define GPSRATE 4800
//#define GPSRATE 38400


// GPS parser for 406a
#define BUFFSIZ 90 // plenty big
//char buffer[BUFFSIZ];
char *parseptr;
char buffidx;
uint8_t hour, minute, second, year, month, date;
uint32_t latitude, longitude;
uint8_t groundspeed, trackangle;
char latdir, longdir;
char status;
uint32_t waypoint = 0;
KellyCanbus kellyCanbus = KellyCanbus(1.84);
 
void setup() {
    uint16_t ret;
  Serial.begin(GPSRATE);
  mySerial.begin(GPSRATE);
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
  Serial.println("ECU Reader");  /* For debug use */
  
  sLCD.begin(9600);              /* Setup serial LCD and clear the screen */
  clear_lcd();
 
 /*
  sLCD.print("D:KLY  U:GPS");
  sLCD.print(COMMAND,BYTE);
  sLCD.print(LINE1,BYTE); 
  sLCD.print("L:SD   R:LOG");
  
  sLCD.print(COMMAND,BYTE);
  sLCD.print(LINE1,BYTE); 

  while(1)
  {
    
    if (digitalRead(UP) == 0){
      Serial.println("gps");
      sLCD.print("GPS");
      gps_test();
    }
    
    if (digitalRead(DOWN) == 0) {
      sLCD.print("KLY");
      Serial.println("KLY");
      break;
    }
    
    if (digitalRead(LEFT) == 0) {
    
      Serial.println("SD test");
      sd_test();
    }
    
    if (digitalRead(RIGHT) == 0) {
    
      Serial.println("Logging");
      logging();
    }
    
  }
  */
  
    clear_lcd();
    move_to ( 0, 0 );
    if(kellyCanbus.init()) {
        sLCD.print("CAN Init ok");
    } else {
        sLCD.print("Can't init CAN");
    } 

    /*
    
    memset ( &buffer, 0, 512 );
    iterations = kellyCanbus.getModel(buffer);
    Serial.print ( "Iterations: " );
    Serial.println ( iterations );
    Serial.print ( "Buffer: '" );
    Serial.print ( buffer );
    Serial.println ( "'" );

    clear_lcd();
    sLCD.print(COMMAND,BYTE); 
    sLCD.print(LINE0,BYTE);
    sLCD.print ( "Model: " );
    sLCD.print ( buffer );

    memset ( &canbusMessage, 0, sizeof ( tCAN ) );
    Serial.print ( "Message ID Before: " );
    Serial.println ( canbusMessage.id, HEX );
    ret = KellyCanbus.batchRead1 ( &iterations, &canbusMessage );
    Serial.print ( "Iterations: " );
    Serial.println ( iterations );
    Serial.print ( "Message ID: " );
    Serial.println ( canbusMessage.id, HEX );
    Serial.print ( "Message ID2: " );
    Serial.print ( ret, HEX );
    Serial.print ( ", " );
    Serial.println ( ret );
    for ( int i = 0; i < 8; i++ ) {
        Serial.print ( "Data " );
        Serial.print ( i, DEC );
        Serial.print ( ": " );
        Serial.print ( canbusMessage.data[i], DEC );
        Serial.print ( ", 0x" );
        Serial.println ( canbusMessage.data[i], HEX );
    }
    //Serial.print ( "Brake A/D: 0x" );
    //Serial.print ( "TPS A/D: 0x" );
    //Serial.println ( canbusMessage.data[1], HEX );
    //Serial.print ( "Controller Power A/D: 0x" );
    //Serial.println ( canbusMessage.data[2], HEX );
    //Serial.print ( "Vs A/D: 0x" );
    //Serial.println ( canbusMessage.data[3], HEX );
    //Serial.print ( "B+ A/D: 0x" );
    //Serial.println ( canbusMessage.data[4], HEX );
    sLCD.print(COMMAND,BYTE); 
    sLCD.print(LINE1,BYTE);
    sLCD.print ( "RET: " );
    sLCD.print ( ret );
    */

   
  delay(100); 
  iterations = 0;

}
 
void loop() {
    bool gotGPSData = false;
    iterations++;

    kellyCanbus.fetchRuntimeData();
    gotGPSData = read_gps();
    if ( ( millis() - lastMillis ) >= 1000 ) {
        lastMillis = millis();
        move_to ( 0, 0 );
        sLCD.print ( groundspeed * 1.15077945, 2 );
        sLCD.print ( " " );
        sLCD.print ( kellyCanbus.getMPHFromRPM(), 2 );
        sLCD.print ( " " );
        sLCD.print ( (float)iterations / (float)millis() * (float)1000, 1 );
        move_to ( 1, 0 );
        sLCD.print ( "B+: " );
        sLCD.print ( kellyCanbus.getTractionPackVoltage(), 3 );
    }
    
    //Serial.println ( buffer );
    Serial.print ( year, DEC );
    Serial.print ( month, DEC );
    Serial.print ( date, DEC );
    Serial.print ( "_" );
    Serial.print ( hour, DEC );
    Serial.print ( minute, DEC );
    Serial.print ( second, DEC );
    Serial.print ( "," );
    Serial.print ( groundspeed * 1.15077945, 2 );
    Serial.print ( "," );
    Serial.print ( kellyCanbus.getMPHFromRPM(), 2 );
    Serial.print ( "," );
    Serial.print ( kellyCanbus.getTractionPackVoltage(), 3 );
    Serial.print ( "," );
    Serial.print ( lat_str );
    Serial.print ( "," );
    Serial.print ( lon_str );
    Serial.print ( "," );
    Serial.print ( groundspeed, DEC );
    Serial.print ( "," );
    Serial.print ( trackangle, DEC );
    Serial.print ( "," );
    for ( int i = 0; i < 22; i++ ) {
        Serial.print ( kellyCanbus.rawData[i], DEC );
        Serial.print ( "," );
    }
    Serial.println();
   
    //Serial.println ( kellyCanbus.dump() );
 
//  if(Canbus.ecu_req(ENGINE_RPM,buffer) == 1)          /* Request for engine RPM */
//  {
//    sLCD.print(COMMAND,BYTE);                   /* Move LCD cursor to line 0 */
//    sLCD.print(LINE0,BYTE);
//    sLCD.print(buffer);                         /* Display data on LCD */
//   
//    
//  } 
//  digitalWrite(LED3, HIGH);
//   
//  if(Canbus.ecu_req(VEHICLE_SPEED,buffer) == 1)
//  {
//    sLCD.print(COMMAND,BYTE);
//    sLCD.print(LINE0 + 9,BYTE);
//    sLCD.print(buffer);
//   
//  }
//  
//  if(Canbus.ecu_req(ENGINE_COOLANT_TEMP,buffer) == 1)
//  {
//    sLCD.print(COMMAND,BYTE);
//    sLCD.print(LINE1,BYTE);                     /* Move LCD cursor to line 1 */
//    sLCD.print(buffer);
//   
//   
//  }
//  
//  if(Canbus.ecu_req(THROTTLE,buffer) == 1)
//  {
//    sLCD.print(COMMAND,BYTE);
//    sLCD.print(LINE1 + 9,BYTE);
//    sLCD.print(buffer);
//     file.print(buffer);
//  }  
//  Canbus.ecu_req(O2_VOLTAGE,buffer);
//    memset ( &canbusMessage, 0, sizeof ( tCAN ) );
//    iterations = 5;
//    gotMessage = KellyCanbus.getNextMessage ( &iterations, &canbusMessage );
//    if ( gotMessage ) {
//        Serial.print ( "Got a message: Iterations: " );
//        Serial.print ( iterations );
//        Serial.print ( ", ID: " );
//        Serial.println ( canbusMessage.id, HEX );
//    } else {
//        Serial.print ( "Did not get a message in " );
//        Serial.print ( iterations );
//        Serial.println ( " iterations" );
//    }
    //delay(100); 
}


void logging(void)
{
  clear_lcd();
  
//  if(Canbus.init(CANSPEED_500))  /* Initialise MCP2515 CAN controller at the specified speed */
//  {
//    sLCD.print("CAN Init ok");
//  } else
//  {
//    sLCD.print("Can't init CAN");
//  } 
//   
//  delay(500);
//  clear_lcd(); 
//  sLCD.print("Init SD card");  
//  delay(500);
//  clear_lcd(); 
//  sLCD.print("Press J/S click");  
//  sLCD.print(COMMAND,BYTE);
//  sLCD.print(LINE1,BYTE);                     /* Move LCD cursor to line 1 */
//   sLCD.print("to Stop"); 
//  
//  // initialize the SD card at SPI_HALF_SPEED to avoid bus errors with
//  // breadboards.  use SPI_FULL_SPEED for better performance.
//  if (!card.init(SPI_HALF_SPEED,9)) error("card.init failed");
//  
//  // initialize a FAT volume
//  if (!volume.init(&card)) error("volume.init failed");
//  
//  // open the root directory
//  if (!root.openRoot(&volume)) error("openRoot failed");
//
//  // create a new file
//  char name[] = "WRITE00.TXT";
//  for (uint8_t i = 0; i < 100; i++) {
//    name[5] = i/10 + '0';
//    name[6] = i%10 + '0';
//    if (file.open(&root, name, O_CREAT | O_EXCL | O_WRITE)) break;
//  }
//  if (!file.isOpen()) error ("file.create");
//  Serial.print("Writing to: ");
//  Serial.println(name);
//  // write header
//  file.writeError = 0;
//  file.print("READY....");
//  file.println();  
//
//  while(1)    /* Main logging loop */
//  {
//     read_gps();
//     
//     file.print(waypoint++);
//     file.print(',');
//     file.print(lat_str);
//     file.print(',');
//     file.print(lon_str);
//     file.print(',');
//      
//    if(Canbus.ecu_req(ENGINE_RPM,buffer) == 1)          /* Request for engine RPM */
//      {
//        sLCD.print(COMMAND,BYTE);                   /* Move LCD cursor to line 0 */
//        sLCD.print(LINE0,BYTE);
//        sLCD.print(buffer);                         /* Display data on LCD */
//        file.print(buffer);
//         file.print(',');
//    
//      } 
//      digitalWrite(LED3, HIGH);
//   
//      if(Canbus.ecu_req(VEHICLE_SPEED,buffer) == 1)
//      {
//        sLCD.print(COMMAND,BYTE);
//        sLCD.print(LINE0 + 9,BYTE);
//        sLCD.print(buffer);
//        file.print(buffer);
//        file.print(','); 
//      }
//      
//      if(Canbus.ecu_req(ENGINE_COOLANT_TEMP,buffer) == 1)
//      {
//        sLCD.print(COMMAND,BYTE);
//        sLCD.print(LINE1,BYTE);                     /* Move LCD cursor to line 1 */
//        sLCD.print(buffer);
//         file.print(buffer);
//       
//      }
//      
//      if(Canbus.ecu_req(THROTTLE,buffer) == 1)
//      {
//        sLCD.print(COMMAND,BYTE);
//        sLCD.print(LINE1 + 9,BYTE);
//        sLCD.print(buffer);
//         file.print(buffer);
//      }  
//    //  Canbus.ecu_req(O2_VOLTAGE,buffer);
//       file.println();  
//  
//       digitalWrite(LED3, LOW); 
// 
//       if (digitalRead(CLICK) == 0){  /* Check for Click button */
//           file.close();
//           Serial.println("Done");
//           sLCD.print(COMMAND,BYTE);
//           sLCD.print(CLEAR,BYTE);
//     
//           sLCD.print("DONE");
//          while(1);
//        }
//
//  }
// 
// 
 
 
}
     

void sd_test(void)
{
 clear_lcd(); 
 sLCD.print("SD test"); 
 Serial.println("SD card test");
   
     // initialize the SD card at SPI_HALF_SPEED to avoid bus errors with
  // breadboards.  use SPI_FULL_SPEED for better performance.
  if (!card.init(SPI_HALF_SPEED,9)) error("card.init failed");
  
  // initialize a FAT volume
  if (!volume.init(&card)) error("volume.init failed");
  
  // open root directory
  if (!root.openRoot(&volume)) error("openRoot failed");
  // open a file
  if (file.open(&root, "LOGGER00.CSV", O_READ)) {
    Serial.println("Opened PRINT00.TXT");
  }
  else if (file.open(&root, "WRITE00.TXT", O_READ)) {
    Serial.println("Opened WRITE00.TXT");    
  }
  else{
    error("file.open failed");
  }
  Serial.println();
  
  // copy file to serial port
  int16_t n;
  uint8_t buf[7];// nothing special about 7, just a lucky number.
  while ((n = file.read(buf, sizeof(buf))) > 0) {
    for (uint8_t i = 0; i < n; i++) Serial.print(buf[i]);
  
  
  }
 clear_lcd();  
 sLCD.print("DONE"); 

  
 while(1);  /* Don't return */ 
    

}
bool read_gps(void)
{
 uint32_t tmp;

  unsigned char i;
  unsigned char exit = 0;

  while( exit == 0)
  { 
    
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
      //  Serial.println(" ");
      //  Serial.println(lat_str);
       
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

}

      
      

void gps_test(void){
  uint32_t tmp;
  uint32_t lat;
  unsigned char i;
  
  while(1){
  
   readline();
  
  // check if $GPRMC (global positioning fixed data)
  if (strncmp(buffer, "$GPRMC",6) == 0) {
    
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
     Serial.println("\nlat_str ");
     Serial.println(lat_str);
   
  
    // grab latitude & long data
    // latitude
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
    
    //Serial.println(latdir);
    
    // longitude
    parseptr = strchr(parseptr, ',')+1;
  
    for(i=0;i<12;i++)
    {
      lon_str[i] = parseptr[i];
    }
    lon_str[13] = 0;
    
    Serial.println(lon_str);
   
  
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
    
    Serial.print("\nTime: ");
    Serial.print(hour, DEC); Serial.print(':');
    Serial.print(minute, DEC); Serial.print(':');
    Serial.print(second, DEC); Serial.print(' ');
    Serial.print("Date: ");
    Serial.print(month, DEC); Serial.print('/');
    Serial.print(date, DEC); Serial.print('/');
    Serial.println(year, DEC);
    
    sLCD.print(COMMAND,BYTE);
    sLCD.print(0x80,BYTE);
    sLCD.print("La");
   
    Serial.print("Lat"); 
    if (latdir == 'N')
    {
       Serial.print('+');
       sLCD.print("+");
    }
    else if (latdir == 'S')
    {  
       Serial.print('-');
       sLCD.print("-");
    }
     
    Serial.print(latitude/1000000, DEC); Serial.print('\°', BYTE); Serial.print(' ');
    Serial.print((latitude/10000)%100, DEC); Serial.print('\''); Serial.print(' ');
    Serial.print((latitude%10000)*6/1000, DEC); Serial.print('.');
    Serial.print(((latitude%10000)*6/10)%100, DEC); Serial.println('"');
    
    
    
    sLCD.print(latitude/1000000, DEC); sLCD.print(0xDF, BYTE); sLCD.print(' ');
    sLCD.print((latitude/10000)%100, DEC); sLCD.print('\''); //sLCD.print(' ');
    sLCD.print((latitude%10000)*6/1000, DEC); sLCD.print('.');
    sLCD.print(((latitude%10000)*6/10)%100, DEC); sLCD.print('"');
    
    sLCD.print(COMMAND,BYTE);
    sLCD.print(0xC0,BYTE);
    sLCD.print("Ln");
   
      
    Serial.print("Long: ");
    if (longdir == 'E')
    {
       Serial.print('+');
       sLCD.print('+');
    }
    else if (longdir == 'W')
    { 
       Serial.print('-');
       sLCD.print('-');
    }
    Serial.print(longitude/1000000, DEC); Serial.print('\°', BYTE); Serial.print(' ');
    Serial.print((longitude/10000)%100, DEC); Serial.print('\''); Serial.print(' ');
    Serial.print((longitude%10000)*6/1000, DEC); Serial.print('.');
    Serial.print(((longitude%10000)*6/10)%100, DEC); Serial.println('"');
    
    sLCD.print(longitude/1000000, DEC); sLCD.print(0xDF, BYTE); sLCD.print(' ');
    sLCD.print((longitude/10000)%100, DEC); sLCD.print('\''); //sLCD.print(' ');
    sLCD.print((longitude%10000)*6/1000, DEC); sLCD.print('.');
    sLCD.print(((longitude%10000)*6/10)%100, DEC); sLCD.print('"');
     
  }
  
 //   Serial.println("Lat: ");
 //   Serial.println(latitude);
  
 //   Serial.println("Lon: ");
 //   Serial.println(longitude);
  }



}

bool readline(void) {
  char c;
  bool keeplooping = true;
  int available;
  
  buffidx = 0; // start at begninning
  if ( ! mySerial.available() ) {
      return false;
  }
  while (1) {
      c=mySerial.read();
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
  sLCD.print(COMMAND,BYTE);
  sLCD.print(CLEAR,BYTE);
}  

void move_to ( int row, int column ) {
    int commandChar;
    commandChar = baseChars16Column[row];
    commandChar += column;
        /* set the high 7 bit to 1 per the spec */
    commandChar |= 0x80;
    sLCD.print(COMMAND,BYTE);
    sLCD.print(commandChar,BYTE);
}
