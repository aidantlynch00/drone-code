#pragma once
#include <wiringPi.h>
#include "GPIO.h"

class SoftPWM_GPIO : public GPIO {
	public:
		SoftPWM_GPIO(int bcm_pin, int range);
		~SoftPWM_GPIO();

		void setValue(int _value);
};