/**
 * 
 *
 * Copyright (c) 2008-2009  All rights reserved.
 */
#include <WProgram.h>
#include "WConstants.h"
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "pins_arduino.h"
#include <inttypes.h>
#include "global.h"
#include "mcp2515.h"
#include "defaults.h"
#include "KellyCanbus.h"

uint8_t CCP_A2D_BATCH_READ1_DATA[1] = { 0x1b };
uint8_t CCP_A2D_BATCH_READ2_DATA[1] = { 0x1a };
uint8_t CCP_MONITOR1_DATA[1] = { 0x33 };
uint8_t CCP_MONITOR2_DATA[1] = { 0x37 };

uint8_t responseData[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

KellyCanbus::KellyCanbus(float divider) {
    this->divider = divider;
    this->count = 0;
}

char KellyCanbus::init() {
    return mcp2515_init(CANSPEED_1000);
}

void KellyCanbus::fetchAllRuntimeData() {
    getCCP_A2D_BATCH_READ1();
    getCCP_A2D_BATCH_READ2();
    getCCP_MONITOR1();
    getCCP_MONITOR2();

    iAvg = (float)iCumulative / count;
    vAvg = (float)vCumulative / count;
    wAvg = iAvg * vAvg;
}

void KellyCanbus::getCCP_A2D_BATCH_READ1() {
    memset ( responseData, 0, 8 );
    bool status = canRequest ( CCP_A2D_BATCH_READ1_DATA, responseData );
    memcpy ( rawData + CCP_A2D_BATCH_READ1_OFFSET, responseData, 5 );
    //brakeAnalogRaw = responseData[0];
    //throttleAnalogRaw = responseData[1];
    //controllerVoltageRaw = responseData[2];
    //fiveVoltVoltageRaw = responseData[3];
    tractionPackVoltageRaw = responseData[4];
}

void KellyCanbus::getCCP_A2D_BATCH_READ2() {
    uint8_t vMax = 0;
    uint8_t iMax = 0;
    memset ( responseData, 0, 8 );
    bool status = canRequest ( CCP_A2D_BATCH_READ2_DATA, responseData );
    memcpy ( rawData + CCP_A2D_BATCH_READ2_OFFSET, responseData, 6 );

    for ( int i = 0; i < 3; i++ ) {
        if ( responseData[i] > iMax ) {
            iMax = responseData[i];
        }
        if ( responseData[i+3] > vMax ) {
            vMax = responseData[i+3];
        }
    }
        // Deal with weird situation where current is 0, 1, or 2, and
        // voltage between 62 and 70 or so.  if so, set vMax to 0.
    if ( ( iMax <= 2 ) && ( ( vMax >=62 ) && ( vMax <= 69 ) ) ) {
        vMax = 0;
    }
    count++;
    iCumulative += iMax;
    vCumulative += vMax;
}

void KellyCanbus::getCCP_MONITOR1() {
    memset ( responseData, 0, 8 );
    bool status = canRequest ( CCP_MONITOR1_DATA, responseData );
    memcpy ( rawData + CCP_MONITOR1_OFFSET, responseData, 6 );
    //pwm = responseData[0];
    //enableMotorRotation = responseData[1];
    //motorTemperature = responseData[2];
    //controllerTemperature = responseData[3];
    //highMosfetTemp = responseData[4];
    //lowMosfetTemp = responseData[5];
}

void KellyCanbus::getCCP_MONITOR2() {
    memset ( responseData, 0, 8 );
    bool status = canRequest ( CCP_MONITOR2_DATA, responseData );
    memcpy ( rawData + CCP_MONITOR2_OFFSET, responseData, 5 );
    rpm = responseData[0] << 8 | responseData[1];
    //percentRatedCurrent = responseData[2];
    //errorCode = responseData[3] << 8 | responseData[4];
}

float KellyCanbus::getMPHFromRPM() {
    return (float)rpm * 80.296 * 60 / 12 / 5280 / 2;
}

float KellyCanbus::getTractionPackVoltage() {
    return (float)tractionPackVoltageRaw / divider;
}

void KellyCanbus::resetMotorInfo() {
    iCumulative = 0;
    vCumulative = 0;
    count = 0;
}

bool KellyCanbus::canRequest(uint8_t requestData[8], uint8_t *responseData) {
    tCAN request;
    tCAN response;
    int iterations;
    uint8_t address;
    request.id = 0x6B;
    request.header.rtr = 0;
    request.header.length = 1;
    memcpy ( request.data, requestData, 1 );
    mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
    address = mcp2515_send_message(&request);
    
    while(iterations < 40000) {
        iterations++;
        if (mcp2515_check_message()) 
        {
            if (mcp2515_get_message(&response)) 
            {
                memcpy ( responseData, response.data, 8 );
                return true;
            }
        }
    }
    return false;
}

/*
String KellyCanbus::dump() {
    String line;
    char floatStr[7];
    line += String (millis(), DEC);
    line += ",";
    //dtostrf ( 12.2, 6, 3, floatStr );
    line += floatStr;
    line += ",";
//
//    dtostrf ( motorInfo.iAAvg, 7, 3, floatStr );
//    line += floatStr;
//    line += ",";
//    
//    dtostrf ( motorInfo.iBAvg, 7, 3, floatStr );
//    line += floatStr;
//    line += ",";
//
//    dtostrf ( motorInfo.iCAvg, 7, 3, floatStr );
//    line += floatStr;
//    line += ",";
//
//    dtostrf ( motorInfo.vAAvg, 7, 3, floatStr );
//    line += floatStr;
//    line += ",";
//    
//    dtostrf ( motorInfo.vBAvg, 7, 3, floatStr );
//    line += floatStr;
//    line += ",";
//    
//    dtostrf ( motorInfo.vCAvg, 7, 3, floatStr );
//    line += floatStr;
//    line += ",";
    return line;
}
*/
// 
// 
// void KellyCanbus::sendMessage(uint8_t data[8]);
//     tCAN message;
//     uint8_t address;
//     char message_ok = 0;
//     // Prepair message
//     message.id = 0x6B;
//     message.header.rtr = 0;
//     message.header.length = 1;
//     memcpy ( message.data, data, 8 );
//     
//     mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
//     address = mcp2515_send_message(&message);
//     return;
// }
// 
// void KellyCanbus::getModel() {
//     
//     while(*iterations < 40000) {
//         *iterations++;
//         if (mcp2515_check_message()) 
//         {
//             if (mcp2515_get_message(outMessage)) 
//             {
//                 return outMessage->id;
//             }
//         }
//     }
//     return 1;
// }
// 
// 
// 
// void KellyCanbus::foo() {
//     Serial.println ( "Hello foo" );
// }
// int KellyCanbus::getModel(char *buffer) {
//     tCAN message;
//     int iterations = 0;
//     uint8_t address;
//     char message_ok = 0;
//     char model[9];
//     // Prepair message
//     message.id = 0x6B;
//     message.header.rtr = 0;
//     message.header.length = 8;
//     message.data[0] = 0xF2;
//     message.data[1] = 64;
//     message.data[2] = 0x08;
//     message.data[3] = 0x00;
//     message.data[4] = 0x00;
//     message.data[5] = 0x00;
//     message.data[6] = 0x00;
//     message.data[7] = 0x00;						
//     //Serial.println ( "abc" );
//     //Serial.println ( "def" );
//     model[0] = 'a';
//     //Serial.println ( "ghi" );
//     model[1] = 'a';
//     model[2] = 'k';
//     model[3] = 't';
//     model[4] = 0x00;
//     model[5] = 0x00;
//     model[6] = 0x00;
//     model[7] = 0x00;
//     model[8] = 0x00;
//     
//     /*
//     buffer[0] = 'y';
//     buffer[1] = 'o';
//     buffer[2] = 'y';
//     buffer[3] = NULL;
//     */
//    
//     
//                     //memcpy ( buffer, model, 8 );
//                     //Serial.println ( "buffer: " );
//                     //return buffer;
//                     //Serial.println ( buffer );
//                     // return "foobar";
//     mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
// //		SET(LED2_HIGH);	
//     address = mcp2515_send_message(&message);
//     
//     while(iterations < 4000) {
//         iterations++;
//         if (mcp2515_check_message()) 
//         {
//             if (mcp2515_get_message(&message)) 
//             {
//                 if(message.id == 0x73) // Check message is the reply and its the right PID 
//                 {
//                     memcpy ( buffer, message.data, 8 );
//                     return iterations;
//                 } 
//                 else 
//                 {
//                     buffer[0] = message.id;
//                     return iterations;
//                 }
//             }
//         }
//     }
//     return iterations;
// }
// 
// uint16_t KellyCanbus::batchRead1(int *iterations, tCAN *outMessage) {
//     tCAN message;
//     uint8_t address;
//     char message_ok = 0;
//     char model[9];
//     // Prepair message
//     message.id = 0x6B;
//     message.header.rtr = 0;
//     message.header.length = 1;
//     message.data[0] = 0x1b;
//     message.data[1] = 0x00;
//     message.data[2] = 0x08;
//     message.data[3] = 0x00;
//     message.data[4] = 0x00;
//     message.data[5] = 0x00;
//     message.data[6] = 0x00;
//     message.data[7] = 0x00;						
//     
//     mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
// //		SET(LED2_HIGH);	
//     address = mcp2515_send_message(&message);
//     
//     while(*iterations < 40000) {
//         *iterations++;
//         if (mcp2515_check_message()) 
//         {
//             if (mcp2515_get_message(outMessage)) 
//             {
//                 return outMessage->id;
//             }
//         }
//     }
//     return 1;
// }
// 
// bool KellyCanbus::getNextMessage(int *iterations, tCAN *message) {
//     
//         // Set listen only mode.
//     
//     while(*iterations < 4000) {
//         *iterations++;
//         if (mcp2515_check_message()) 
//         {
//             if (mcp2515_get_message(message)) 
//             {
//                 return true;
//             }
//         }
//     }
//     return false;
// }
// 
