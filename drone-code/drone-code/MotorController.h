#pragma once
#include "SoftPWM_GPIO.h"

class MotorController {

private:
	SoftPWM_GPIO* clockwise;
	SoftPWM_GPIO* counterclockwise;
	SoftPWM_GPIO* enable;
};