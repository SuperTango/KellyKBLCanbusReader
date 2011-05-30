// A simple sketch to read GPS data and parse the $GPRMC string 
// see http://www.ladyada.net/make/gpsshield for more info

#include <NewSoftSerial.h>

NewSoftSerial mySerial =  NewSoftSerial(4, 5);
NewSoftSerial sLCD =  NewSoftSerial(3, 14); /* Serial LCD is connected on pin 14 (Analog input 0) */
#define COMMAND 0xFE
//#define powerpin 4

#define GPSRATE 4800
//#define GPSRATE 38400


// GPS parser for 406a
#define BUFFSIZ 90 // plenty big
char buffer[BUFFSIZ];
char *parseptr;
char buffidx;
uint8_t hour, minute, second, year, month, date;
uint32_t latitude, longitude;
uint8_t groundspeed, trackangle;
char latdir, longdir;
char status;

void setup() 
{ 
 // if (powerpin) {
 //   pinMode(powerpin, OUTPUT);
 // }
  pinMode(13, OUTPUT);
  Serial.begin(GPSRATE);
  mySerial.begin(GPSRATE);
   
  // prints title with ending line break 
  Serial.println("GPS parser"); 
  sLCD.begin(9600);
  sLCD.print(0xFE,BYTE);
  sLCD.print(0x01,BYTE);
  
//   digitalWrite(powerpin, LOW);         // pull low to turn on!
} 
 
 
void loop() 
{ 
  uint32_t tmp;
  
  Serial.print("\n\rread: ");
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
    Serial.println(second, DEC);
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
     {  Serial.print('-');
          sLCD.print("-");
     }
    Serial.print(latitude/1000000, DEC); Serial.print('\°', BYTE); Serial.print(' ');
    
    sLCD.print(latitude/1000000, DEC); sLCD.print(0xDF, BYTE); sLCD.print(' ');
    
    Serial.print((latitude/10000)%100, DEC); Serial.print('\''); Serial.print(' ');
     sLCD.print((latitude/10000)%100, DEC); sLCD.print('\''); //sLCD.print(' ');
     
    Serial.print((latitude%10000)*6/1000, DEC); Serial.print('.');
    sLCD.print((latitude%10000)*6/1000, DEC); sLCD.print('.');
    
    Serial.print(((latitude%10000)*6/10)%100, DEC); Serial.println('"');
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
     sLCD.print(longitude/1000000, DEC); sLCD.print(0xDF, BYTE); sLCD.print(' ');
     
    Serial.print((longitude/10000)%100, DEC); Serial.print('\''); Serial.print(' ');
  sLCD.print((longitude/10000)%100, DEC); sLCD.print('\''); //sLCD.print(' ');
  
    Serial.print((longitude%10000)*6/1000, DEC); Serial.print('.');
    sLCD.print((longitude%10000)*6/1000, DEC); sLCD.print('.');
    
    Serial.print(((longitude%10000)*6/10)%100, DEC); Serial.println('"');
   sLCD.print(((longitude%10000)*6/10)%100, DEC); sLCD.print('"');
   
   
  }
  //Serial.println(buffer);
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

void readline(void) {
  char c;
  
  buffidx = 0; // start at begninning
  while (1) {
      c=mySerial.read();
      if (c == -1)
        continue;
      Serial.print(c);
      if (c == '\n')
        continue;
      if ((buffidx == BUFFSIZ-1) || (c == '\r')) {
        buffer[buffidx] = 0;
        return;
      }
      buffer[buffidx++]= c;
  }
}
