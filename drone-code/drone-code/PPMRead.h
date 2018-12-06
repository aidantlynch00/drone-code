#pragma once
#include <wiringPi.h>
#include <stdint.h>

#define PERIOD			20
#define SAMPLING_PERIOD 1
#define RC_CHANNEL_1	17
#define RC_CHANNEL_2	18
#define BAUD_RATE		2

class PPMRead {
	public:
		PPMRead();
		~PPMRead();
		bool syncListener();
		int valueReader(int pin);

	private:
		uint32_t startTime;
		uint32_t currentTime;
		uint32_t positionTime;
		int cycles;
		bool synced;


};