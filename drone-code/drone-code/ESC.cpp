#include <wiringPi.h>
#include <softPwm.h>
#include "ESC.h"

ESC::ESC() {
	motors = new SoftPWM_GPIO[4];
}

ESC::~ESC() {

}

