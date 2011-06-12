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


class KellyCanbusClass
{
  public:

	KellyCanbusClass();
        char init(unsigned char);
        int getModel(char *buffer);
        bool getNextMessage(int *iterations, tCAN *message);
        uint16_t batchRead1(int *iterations, tCAN *outMessage);
        void foo();
private:
	
};
extern KellyCanbusClass KellyCanbus;

#endif

