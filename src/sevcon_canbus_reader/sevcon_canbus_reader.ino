#include <SoftwareSerial.h>
#include <Time.h>
#include <avr/pgmspace.h>
#include "KellyCanbus.h"

#include "CanOpenLite.h"

/* Define Joystick connection */
#define UP     A1
#define RIGHT  A2
#define DOWN   A3
#define CLICK  A4
#define LEFT   3
#define CHIP_SELECT 9

#define LCD_COMMAND 0xFE
#define LCD_CLEAR   0x01
uint8_t baseChars20Column[4] = { 0, 64, 20, 84 };

int LED2 = 8;
int LED3 = 7;
unsigned long currentMillis;

KellyCanbus kellyCanbus = KellyCanbus(1.84);
CanOpenLite canOpenLite = CanOpenLite();
tCAN request;
tCAN response;

SoftwareSerial lcdSerial = SoftwareSerial(-1, 6);

SdoMessage sdoMsg;


uint8_t rpmFromController = 0;
float throttlePercent = 0.0;
float batteryVoltage = 0.0;
float motorVoltage = 0.0;
float batteryCurrent = 0.0;
float motorCurrent = 0.0;
float motorTempFromController = 0.0;
float motorTempFromThermistor = 0.0;

typedef struct {
    uint16_t index;
    uint8_t subIndex;
    float scalingFactor;
    float value;
    char *description;
} ObjectDictionaryEntry;

ObjectDictionaryEntry throttleValueOD = { 0x2620, 0, 0.0000305185094759972, 0, "throttleValue" };
ObjectDictionaryEntry batteryVoltageOD = { 0x5100, 1, 0.0625, 0, "batteryVoltage" };
ObjectDictionaryEntry batteryCurrentOD = { 0x5100, 2, 0.0625, 0, "batteryCurrent" };
ObjectDictionaryEntry motorVoltageOD = { 0x4600, 0xD, 0.0625, 0, "motorVoltage" };
ObjectDictionaryEntry motorCurrentOD = { 0x4600, 0xC, 1, 0, "motorCurrent" };
ObjectDictionaryEntry heatsinkTempOD = { 0x5100, 0x4, 1, 0, "heatsinkTemp" };
ObjectDictionaryEntry motorTempOD = { 0x4600, 0x3, 1, 0, "motorTemp" };

ObjectDictionaryEntry *idsToFetch[] = { &throttleValueOD, &batteryVoltageOD, &batteryCurrentOD, &motorVoltageOD, &motorCurrentOD, &heatsinkTempOD, &motorTempOD, NULL };

unsigned long lastCanRequestMillis = 0;
uint8_t idCount = 0;
uint32_t iterations = 0;

void lcd_clear(void)
{
    lcdSerial.write(LCD_COMMAND);
    lcdSerial.write(LCD_CLEAR);
}  

void lcd_move_to ( int row, int column ) {
    int commandChar;
    commandChar = baseChars20Column[row];
    commandChar += column;
        /* set the high 7 bit to 1 per the spec */
    commandChar |= 0x80;
    lcdSerial.write(LCD_COMMAND);
    lcdSerial.write(commandChar);
}

void setup() {
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
    lcdSerial.begin(9600);              /* Setup serial LCD and clear the screen */

    if(kellyCanbus.init()) {
        Serial.println ( "canbus init ok" );
    } else {
        Serial.println ( "canbus init failed" );
    } 
    lcd_clear();
    lcd_move_to ( 1, 5 );
    lcdSerial.begin(9600);              /* Setup serial LCD and clear the screen */
    lcdSerial.print ( "New TangoLogger" );
    lcd_move_to ( 2, 0 );
    lcdSerial.print ( sizeof ( SdoMessage ), DEC );
    delay ( 500 );
}

void loop() {
    currentMillis = millis();
    iterations++;

        // send request
    if ( currentMillis - lastCanRequestMillis > 100 ) {
        ObjectDictionaryEntry *entry = idsToFetch[idCount];
        if ( entry == NULL ) {
            idCount = 0;
            entry = idsToFetch[idCount];
        }
        request.id = 0x601;
        request.header.rtr = 0;
        request.header.length = 8;
        sdoMsg.type = 2;
        sdoMsg.length = 0;
        sdoMsg.index = entry->index;
        sdoMsg.subIndex = entry->subIndex;
        sdoMsg.data = 0x0;
        canOpenLite.sdoMessageToBuffer ( &sdoMsg, request.data );
        /*
        Serial.print ( "millis: " );
        Serial.print ( currentMillis, DEC );
        Serial.print ( ", iterations: " );
        Serial.print ( iterations, DEC );
        Serial.print ( ", " );
        Serial.print ( entry->description );
        Serial.print ( ", Sending Request" );
        Serial.print ( ", idCount: " );
        Serial.println ( idCount, DEC );
        */
        mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
        mcp2515_send_message(&request);
        idCount++;
    }

    
    while (mcp2515_check_message()) {
        memset ( &response, 0, sizeof(tCAN));
        if (mcp2515_get_message(&response)) {
            if ( response.id != 0x423 && response.id != 0x80 ) {
                if ( response.id == 0x581 ) {
                    /*
                    Serial.print ( "COB-ID: " );
                    Serial.print ( response.id, HEX );
                    Serial.print ( ", rtr: " );
                    Serial.print ( response.header.rtr, HEX );
                    Serial.print ( ", length: " );
                    Serial.print ( response.header.length, DEC );
                    Serial.print ( ", data: " );
                    */
                    for ( int i = 0; i < response.header.length; i++ ) {
                        /*
                        Serial.print ( response.data[i], HEX );
                        Serial.print ( " " );
                        */
                    }
                    canOpenLite.sdoMessageFromBuffer ( &sdoMsg, response.data );
                    for ( int i = 0; idsToFetch[i] != NULL ; i++ ) {
                        ObjectDictionaryEntry *ode = idsToFetch[i];
                        if ( ( sdoMsg.index == ode->index ) && ( sdoMsg.subIndex == ode->subIndex ) ) {
                            currentMillis = millis();
                            ode->value = sdoMsg.data * ode->scalingFactor;
                            Serial.print ( "millis: " );
                            Serial.print ( currentMillis, DEC );
                            Serial.print ( ", iterations: " );
                            Serial.print ( iterations, DEC );
                            Serial.print ( ", " );
                            Serial.print ( ode->description );
                            Serial.print ( ", value: " );
                            Serial.println ( ode->value );
                            break;
                        }
                    }
                }
            }
        }
    }
}


