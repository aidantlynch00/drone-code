#pragma once
#include <wiringPi.h>

#define PERIOD 20
#define RC_CHANNEL_1 17
#define RC_CHANNEL_2 18

class PPMRead {
	public:
		PPMRead();
		~PPMRead();
		bool syncListener();
		int valueReader(int pin);
};