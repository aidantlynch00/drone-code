#pragma once
#include <wiringPi.h>
#include <stdint.h>
#include <string>

#define PERIOD			20
#define SAMPLING_PERIOD 1
#define BITS			8

using namespace std;

class RC_Code {
public:
	RC_Code(int pin);
	~RC_Code();
	void read();

private:
	int pin;
	void calc_input(int channel, int pin);
	void setup();
	void rc_read_values();
};