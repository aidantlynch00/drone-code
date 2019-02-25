#pragma once
#include <stdint.h>
#include "SoftPWM_GPIO.h"

class ESC {

public:
	//Change these values based on the specs of the ESC (in us)
	static const int MIN_PULSE_WIDTH = 1000;
	static const int MAX_PULSE_WIDTH = 1999;

private:
	double lowRange, highRange;
	uint32_t pwm;
	SoftPWM_GPIO* pin;

public:
	ESC(int bcm_pin);
	~ESC();
	void setPWM(int _pwm);
	int getPWM();
};
