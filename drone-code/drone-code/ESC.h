#pragma once
#include <stdint.h>
#include "SoftPWM_GPIO.h"

class ESC {
	SoftPWM_GPIO* motors[4];

public:
	ESC();
	~ESC();
	void setPWM(int motor, int pwm);
};
