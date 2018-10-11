#pragma once
#include "SoftPWM_GPIO.h"

class ESC_Controller {

public:
	//Change these values based on the specs of the ESC (in us)
	static const int MIN_PULSE_WIDTH = 1000;
	static const int MAX_PULSE_WIDTH = 1999;

private:
	double lowRange, highRange;
	int speed;
	SoftPWM_GPIO* pin;

public:
	ESC_Controller(int bcm_pin);
	~ESC_Controller();
	void setSpeed(int speed);
	int getSpeed();
};
