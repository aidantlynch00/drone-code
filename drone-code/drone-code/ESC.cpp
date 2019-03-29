#include <wiringPi.h>
#include <softPwm.h>
#include "ESC.h"

ESC::ESC() {
	motors[0] = new SoftPWM_GPIO(6);
	motors[1] = new SoftPWM_GPIO(13);
	motors[2] = new SoftPWM_GPIO(19);
	motors[3] = new SoftPWM_GPIO(26);
}

ESC::~ESC() {
	delete[] motors;
}

void ESC::setPWM(int motor, int pwm) {

}

