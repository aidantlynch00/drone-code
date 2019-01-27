#pragma once
#include <wiringPi.h>
#include <stdint.h>
#include <string>

#define PERIOD			20
#define SAMPLING_PERIOD 1
#define BITS			8

using namespace std;

class RC {
public:
	RC();
	//~RC();
	void read(void);
	uint32_t* getValues();

private:
	int pin;
	void setup();
};
