#pragma once
#include <wiringPi.h>
#include <stdint.h>
#include <string>

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
