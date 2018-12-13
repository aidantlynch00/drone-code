#pragma once
#include <wiringPi.h>
#include <stdint.h>
#include <string>

#define PERIOD			20
#define SAMPLING_PERIOD 1
#define BITS			8

using namespace std;

class PPMRead {
	public:
		PPMRead(int pin);
		~PPMRead();
		bool syncListener();
		string valueReader(int pin);
		int binaryToDeci(string value);
		int finalValues(int pin);
		uint32_t discoverPeriod(int pin);

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