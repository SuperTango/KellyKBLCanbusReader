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

#define CCP_A2D_BATCH_READ1_OFFSET 0
#define CCP_A2D_BATCH_READ2_OFFSET 5
#define CCP_MONITOR1_OFFSET 11
#define CCP_MONITOR2_OFFSET 17
#define MOTOR_TEMP CCP_MONITOR1_OFFSET + 2

class KellyCanbus
{
    public:
	KellyCanbus( float );
        char init();
        void fetchAllRuntimeData();
        void getCCP_A2D_BATCH_READ1();
        void getCCP_A2D_BATCH_READ2();
        void getCCP_MONITOR1();
        void getCCP_MONITOR2();
        float getTractionPackVoltage();
        float getMPHFromRPM();
        String dump();
        bool canRequest (uint8_t[8], uint8_t*);
        uint8_t rawData[22];
        uint16_t iCumulative;
        uint16_t vCumulative;
        float iAvg;
        float vAvg;
        float wAvg;
        void resetMotorInfo();
        uint8_t count;
    private:
        float divider;
        //uint8_t brakeAnalogRaw;
        //uint8_t throttleAnalogRaw;
        //uint8_t controllerVoltageRaw;
        //uint8_t fiveVoltVoltageRaw;
        uint8_t tractionPackVoltageRaw;
        //uint8_t pwm;
        //uint8_t enableMotorRotation;
        //uint8_t motorTemperature;
        //uint8_t controllerTemperature;
        //uint8_t highMosfetTemp;
        //uint8_t lowMosfetTemp;
        uint16_t rpm;
        //uint8_t percentRatedCurrent;
        //uint16_t errorCode;

	
};

#endif

