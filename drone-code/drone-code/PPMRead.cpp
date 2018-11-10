#include "PPMRead.h"
#include <iostream>
#include <wiringPi.h>
#include <string>
#include <cmath>

using namespace std;

PPMRead::PPMRead(int pin)
{
	pinMode(pin, INPUT);
	this->pin = pin;
	cycle = 1;
	startTime = millis();
	currentTime = startTime;
	positionTime = currentTime - startTime;
	currentState = LOW;
	/*if (syncListener()) {
		int count = 0
		while (count < 100) {
			if(finalValues(pin) == HIGH)			//discover values first

		}
	}*/
}

bool PPMRead::syncListener()
{
	bool signalRecieved = false;

	while (!signalRecieved) {
		
	}

	return false;
}

string PPMRead::valueReader()		//Reads 8 bits of information to later convert to decimal or hexaDecimal
{
	string strungBits = "";

	for (int x = 0; x < BITS; x++) {
		strungBits += to_string(readPeriod());
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

int PPMRead::finalValues()
{
	return binaryToDeci(valueReader());
}

uint32_t PPMRead::discoverPeriod()
{
	uint32_t spikeTime;
	currentTime = millis();
	while (true) {
		int value = digitalRead(pin);			//Reads value of pin at millisecond intervals when 
		if(value == HIGH){
			spikeTime = millis();
			break;
		}
	}
	uint32_t spikeTime2;
	while (true) {
		int value = digitalRead(pin);			//Reads value of pin at millisecond intervals when 
		if (value == HIGH) {
			spikeTime2 = millis();
			break;
		}
	}

	return spikeTime2 - spikeTime;
}

int PPMRead::readPeriod()	//Reads 20 milliseconds worth of data
{
	int bit = -1;
	currentTime = millis();
	int milPosition = 0;
	int expectedMilPosition = 0;
	while (milPosition <= PERIOD) {
		cout << milPosition << endl;
		cout << expectedMilPosition << endl;
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
