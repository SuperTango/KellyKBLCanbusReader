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


/* C++ wrapper */
KellyCanbusClass::KellyCanbusClass() {
}

char KellyCanbusClass::init(unsigned char speed) {
    Serial.println ( "Entering init" );
    return mcp2515_init(speed);
}

void KellyCanbusClass::foo() {
    Serial.println ( "Hello foo" );
}
int KellyCanbusClass::getModel(char *buffer) {
    tCAN message;
    int iterations = 0;
    uint8_t address;
    char message_ok = 0;
    char model[9];
    // Prepair message
    message.id = 0x6B;
    message.header.rtr = 0;
    message.header.length = 8;
    message.data[0] = 0xF2;
    message.data[1] = 64;
    message.data[2] = 0x08;
    message.data[3] = 0x00;
    message.data[4] = 0x00;
    message.data[5] = 0x00;
    message.data[6] = 0x00;
    message.data[7] = 0x00;						
    //Serial.println ( "abc" );
    //Serial.println ( "def" );
    model[0] = 'a';
    //Serial.println ( "ghi" );
    model[1] = 'a';
    model[2] = 'k';
    model[3] = 't';
    model[4] = 0x00;
    model[5] = 0x00;
    model[6] = 0x00;
    model[7] = 0x00;
    model[8] = 0x00;
    
    /*
    buffer[0] = 'y';
    buffer[1] = 'o';
    buffer[2] = 'y';
    buffer[3] = NULL;
    */
   
    
                    //memcpy ( buffer, model, 8 );
                    //Serial.println ( "buffer: " );
                    //return buffer;
                    //Serial.println ( buffer );
                    // return "foobar";
    mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
//		SET(LED2_HIGH);	
    address = mcp2515_send_message(&message);
    
    while(iterations < 4000) {
        iterations++;
        if (mcp2515_check_message()) 
        {
            if (mcp2515_get_message(&message)) 
            {
                if(message.id == 0x73) // Check message is the reply and its the right PID 
                {
                    memcpy ( buffer, message.data, 8 );
                    return iterations;
                } 
                else 
                {
                    buffer[0] = message.id;
                    return iterations;
                }
            }
        }
    }
    return iterations;
}

uint16_t KellyCanbusClass::batchRead1(int *iterations, tCAN *outMessage) {
    tCAN message;
    uint8_t address;
    char message_ok = 0;
    char model[9];
    // Prepair message
    message.id = 0x6B;
    message.header.rtr = 0;
    message.header.length = 1;
    message.data[0] = 0x1b;
    message.data[1] = 0x00;
    message.data[2] = 0x08;
    message.data[3] = 0x00;
    message.data[4] = 0x00;
    message.data[5] = 0x00;
    message.data[6] = 0x00;
    message.data[7] = 0x00;						
    
    mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
//		SET(LED2_HIGH);	
    address = mcp2515_send_message(&message);
    
    while(*iterations < 40000) {
        *iterations++;
        if (mcp2515_check_message()) 
        {
            if (mcp2515_get_message(outMessage)) 
            {
                return outMessage->id;
            }
        }
    }
    return 1;
}

bool KellyCanbusClass::getNextMessage(int *iterations, tCAN *message) {
    
        // Set listen only mode.
    
    while(*iterations < 4000) {
        *iterations++;
        if (mcp2515_check_message()) 
        {
            if (mcp2515_get_message(message)) 
            {
                return true;
            }
        }
    }
    return false;
}


KellyCanbusClass KellyCanbus;
