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

uint8_t CCP_A2D_BATCH_READ1_DATA[8] = { 0x1b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t CCP_A2D_BATCH_READ2_DATA[8] = { 0x1a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };


KellyCanbus::KellyCanbus(float divider) {
    this->divider = divider;
}

char KellyCanbus::init() {
    return mcp2515_init(CANSPEED_1000);
}

void KellyCanbus::fetchRuntimeData() {
    getCCP_A2D_BATCH_READ1();

}

void KellyCanbus::getCCP_A2D_BATCH_READ1() {
    uint8_t responseData[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    bool status = request ( CCP_A2D_BATCH_READ1_DATA, responseData );
    brakeAnalogRaw = responseData[0];
    throttleAnalogRaw = responseData[1];
    controllerVoltageRaw = responseData[2];
    fiveVoltVoltageRaw = responseData[3];
    tractionPackVoltageRaw = responseData[4];
}

void KellyCanbus::getCCP_A2D_BATCH_READ2() {
    uint8_t responseData[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    bool status = request ( CCP_A2D_BATCH_READ2_DATA, responseData );
    iA = responseData[0];
    iB = responseData[1];
    iC = responseData[2];
    vA = responseData[3];
    vB = responseData[4];
    vC = responseData[5];
}

uint8_t KellyCanbus::getTractionPackVoltageRaw() {
    return tractionPackVoltageRaw;
}

float KellyCanbus::getTractionPackVoltage() {
    return (float)tractionPackVoltageRaw / divider;
}

bool KellyCanbus::request(uint8_t requestData[8], uint8_t *responseData) {
    tCAN request;
    tCAN response;
    int iterations;
    uint8_t address;
    request.id = 0x6B;
    request.header.rtr = 0;
    request.header.length = 1;
    memcpy ( request.data, requestData, 8 );
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
