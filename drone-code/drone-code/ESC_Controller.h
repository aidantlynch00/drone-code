#pragma once
#include "SoftPWM_GPIO.h"

class ESC_Controller {

private:
	SoftPWM_GPIO* pin;

public:
	ESC_Controller(int bcm_pin);
};
