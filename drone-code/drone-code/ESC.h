#pragma once
#include <stdint.h>
#include "SoftPWM_GPIO.h"

class ESC {
	SoftPWM_GPIO motors[4];

public:
	ESC();
	~ESC();
	void setPWM(int motor, uint32_t pwm);
	uint32_t getPWM(int motor);
};
