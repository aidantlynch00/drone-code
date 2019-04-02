#include "ESC.h"

ESC::ESC() {
	motors[0] = SoftPWM_GPIO{ 6 };
	motors[1] = SoftPWM_GPIO{ 13 };
	motors[2] = SoftPWM_GPIO{ 19 };
	motors[3] = SoftPWM_GPIO{ 26 };
}

ESC::~ESC() {
}

void ESC::setPWM(int motor, uint32_t pwm) {
	motors[motor].setValue(pwm);
}

uint32_t ESC::getPWM(int motor) {
	return motors[motor].getValue();
}

