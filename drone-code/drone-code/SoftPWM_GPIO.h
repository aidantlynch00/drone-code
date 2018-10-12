#pragma once
#include <wiringPi.h>
#include "GPIO.h"

class SoftPWM_GPIO : public GPIO {

	private:
		int range;

	public:
		SoftPWM_GPIO(int bcm_pin, int range = 100);
		~SoftPWM_GPIO();

		int  getDuty();
		int getRange();
		void changeValue(int change);
		void setValue(int _value);
};