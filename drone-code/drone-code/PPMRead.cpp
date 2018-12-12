#include "PPMRead.h"
#include <wiringPi.h>
#include <string>
#include <cmath>

using namespace std;

PPMRead::PPMRead(int pin)
{
	pinMode(pin, INPUT);
	cycle = 1;
	startTime = millis();
	currentTime = startTime;
	positionTime = currentTime - startTime;
	currentState = LOW;
	if (syncListener()) {
		int value = binaryToDeci(valueReader(pin));
	}
}

bool PPMRead::syncListener()
{
	bool signalRecieved = false;

	while (!signalRecieved) {
		
	}

	return false;
}

string PPMRead::valueReader(int pin)		//Reads 8 bits of information to later convert to decimal or hexaDecimal
{
	string strungBits = "";

	for (int x = 0; x < BITS; x++) {
		strungBits += to_string(readPeriod(pin));
	}
	
	return strungBits;
}

int PPMRead::binaryToDeci(string value)
{
	int num = 0;
	for (int x = 0; x < BITS; x++) {
		num += value[x] + pow(2, BITS - x);
	}
	return num;
}

int PPMRead::readPeriod(int pin)	//Reads 20 milliseconds worth of data
{
	int bit = -1;
	currentTime = millis();
	int milPosition = 0;
	int expectedMilPosition = 0;
	while (milPosition <= PERIOD) {
		if (milPosition == expectedMilPosition) {		//Reads value of pin at millisecond intervals when 
			int value = digitalRead(pin);
			
			if (value == HIGH) {
				if (milPosition < PERIOD / 2)
					bit = 0;
				else
					bit = 1;
			}
			expectedMilPosition++;
		}
		uint32_t testTime = millis();

		if (timeDifference(testTime, currentTime)) {
			currentState = testTime;
			milPosition++;
		}
		
	}

	return bit;
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
