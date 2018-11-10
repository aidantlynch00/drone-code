#include <wiringPi.h>
#include <softPWM.h>
#include "ESC_Controller.h"

ESC_Controller::ESC_Controller(int BCM_pin) {
	pin = new SoftPWM_GPIO(BCM_pin);
}

ESC_Controller::~ESC_Controller() {
	delete pin;
}

void ESC_Controller::setPulseWidth(int pulseWidth) {

}