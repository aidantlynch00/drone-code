#include "PPMRead.h"
#include <wiringPi.h>

PPMRead::PPMRead(int pin)
{
	pinMode(pin, INPUT);
	cycle = 1;
	startTime = millis();
	currentTime = startTime;
	positionTime = currentTime - startTime;
	currentState = LOW;
	if (syncListener()) {
		valueReader(pin);
	}
}

bool PPMRead::syncListener()
{
	bool signalRecieved = false;

	while (!signalRecieved) {
		
	}

	return false;
}

int PPMRead::valueReader(int pin)		//Reads 8 bits of information to later convert to decimal or hexaDecimal
{
	int* bits = new int[BITS];

	return 0;
}

int PPMRead::readPeriod(int pin)	//Reads 20 milliseconds worth of data
{
	currentTime = millis();
	int milPosition = 0;
	int expectedMilPosition = 0;
	while (milPosition <= PERIOD) {
		if (milPosition == expectedMilPosition) {		//Reads value of pin at millisecond intervals when 
			int value = digitalRead(pin);
				
			expectedMilPosition++;
		}
		uint32_t testTime = millis();

		if (timeDifference(testTime, currentTime)) {
			currentState = testTime;
			milPosition++;
		}
		
	}

	return 0;
}

bool PPMRead::timeDifference(uint32_t currentTime, uint32_t previousTime) //@return: true if 1 millisecond difference is present
{
	if (currentTime - previousTime == 1)
		return true;

	return false;
}

PPMRead::~PPMRead()
{
}
