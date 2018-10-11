#pragma once
#include <wiringPi.h>
#include "GPIO.h"

class SoftPWM_GPIO : public GPIO {

	private:
		int range;

	public:
		SoftPWM_GPIO(int bcm_pin, int range = 100);
		~SoftPWM_GPIO();

		int  get_duty();
		int getRange();
		void change_value(int change);
		void set_value(int _value);
};