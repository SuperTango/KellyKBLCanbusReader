/**
 * CAN BUS
 *
 * Copyright (c) 2010 Sukkin Pang All rights reserved.
 */

#ifndef canbus__h
#define canbus__h

#define CANSPEED_125 	7		// CAN speed at 125 kbps
#define CANSPEED_250  	3		// CAN speed at 250 kbps
#define CANSPEED_500	1		// CAN speed at 500 kbps
#define CANSPEED_1000	0		// CAN speed at 1Mbps
#include <mcp2515.h>


class KellyCanbus
{
    public:
	KellyCanbus( float );
        char init();
        void fetchRuntimeData();
        void getCCP_A2D_BATCH_READ1();
        void getCCP_A2D_BATCH_READ2();
        float getTractionPackVoltage();
        uint8_t getTractionPackVoltageRaw();
        //int getModel(char *buffer);
        //bool getNextMessage(int *iterations, tCAN *message);
        //uint16_t batchRead1(int *iterations, tCAN *outMessage);
        //float getTractionPackVoltage();
        bool request (uint8_t[8], uint8_t*);
    private:
        char model[8];
        float divider;
        uint8_t brakeAnalogRaw;
        uint8_t throttleAnalogRaw;
        uint8_t controllerVoltageRaw;
        uint8_t fiveVoltVoltageRaw;
        uint8_t tractionPackVoltageRaw;
        uint8_t iA;
        uint8_t iB;
        uint8_t iC;
        uint8_t vA;
        uint8_t vB;
        uint8_t vC;

	
};

#endif

