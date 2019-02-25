#include <wiringPi.h>
#include <softPwm.h>
#include "ESC.h"

ESC::ESC(int BCM_pin) {
	//Setup soft PWM on passed pin with a range of 100 (must contain 10-20)
	pin = new SoftPWM_GPIO(BCM_pin, 100);

	//WiringPi delays by value * 100
	lowRange = MIN_PULSE_WIDTH / 100;
	highRange = MAX_PULSE_WIDTH / 100;

	pwm = 1000;
}

ESC::~ESC() {
	delete pin;
}

//Speed comes in between 0-100
void ESC::setPWM(int _pwm) {
	pwm = _pwm;
	pin->setValue(pwm);
}

int ESC::getPWM() {
	return pwm;
}