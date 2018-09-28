#pragma once
#include <wiringPi.h>
#include "GPIO.h"

class SoftPWM_GPIO : public GPIO {

	private:
		int duty_cycle;

	public:
		SoftPWM_GPIO(int bcm_pin, int duty_cycle = 100);
		~SoftPWM_GPIO();

		//void change_duty(int change);
		int  get_duty();
		//void set_duty(int _duty_cycle);

		void change_value(int change);
		void set_value(int _value);
};