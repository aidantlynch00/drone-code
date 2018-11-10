#pragma once
#include <wiringPi.h>
#include <stdint.h>

#define PERIOD			20
#define SAMPLING_PERIOD 1
#define BITS			8

class PPMRead {
	public:
		PPMRead(int pin);
		~PPMRead();
		bool syncListener();
		int valueReader(int pin);

	private:
		uint32_t startTime;
		uint32_t currentTime;
		uint32_t positionTime;
		int cycle;
		bool synced;
		int currentState;
		int readPeriod(int pin);
		bool timeDifference(uint32_t currentTime, uint32_t previousTime);
};