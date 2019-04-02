#pragma once
#include <wiringPi.h>
#include <stdint.h>
#include "GPIO.h"

class SoftPWM_GPIO : public GPIO {
	public:
		SoftPWM_GPIO(int bcm_pin);
		~SoftPWM_GPIO();

		void setValue(uint32_t _value);
		uint32_t getValue();
};