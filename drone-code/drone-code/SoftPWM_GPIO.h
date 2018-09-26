#pragma once
#include <wiringPi.h>
#include "GPIO.h"

class SoftPWM_GPIO : public GPIO {

	private:
		int duty_cycle;

	public:
		SoftPWM_GPIO();
		~SoftPWM_GPIO();

		void changeDuty(int change);
		int getDuty();
		void setDuty(int duty_cycle);
};