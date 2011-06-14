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

#define MOTOR_SAMPLES 10

typedef struct {
    float iAAvg;
    float iBAvg;
    float iCAvg;
    float vAAvg;
    float vBAvg;
    float vCAvg;
} MotorInfo;

class KellyCanbus
{
    public:
	KellyCanbus( float );
        char init();
        void fetchRuntimeData();
        void getCCP_A2D_BATCH_READ1();
        void getCCP_A2D_BATCH_READ2();
        void getCCP_MONITOR1();
        void getCCP_MONITOR2();
        float getTractionPackVoltage();
        float getMPHFromRPM();
        MotorInfo* getMotorInfo();
        String dump();
        bool canRequest (uint8_t[8], uint8_t*);
        uint8_t rawData[22];
    private:
        char model[8];
        float divider;
        //uint8_t brakeAnalogRaw;
        //uint8_t throttleAnalogRaw;
        //uint8_t controllerVoltageRaw;
        //uint8_t fiveVoltVoltageRaw;
        uint8_t tractionPackVoltageRaw;
        uint8_t iA[MOTOR_SAMPLES];
        uint8_t iB[MOTOR_SAMPLES];
        uint8_t iC[MOTOR_SAMPLES];
        uint8_t vA[MOTOR_SAMPLES];
        uint8_t vB[MOTOR_SAMPLES];
        uint8_t vC[MOTOR_SAMPLES];
        //uint8_t pwm;
        //uint8_t enableMotorRotation;
        //uint8_t motorTemperature;
        //uint8_t controllerTemperature;
        //uint8_t highMosfetTemp;
        //uint8_t lowMosfetTemp;
        uint16_t rpm;
        //uint8_t percentRatedCurrent;
        //uint16_t errorCode;
        MotorInfo motorInfo;

	
};

#endif

