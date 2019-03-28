#pragma once
#include <stdint.h>
#include "SoftPWM_GPIO.h"

class ESC {
	SoftPWM_GPIO* motors;

public:
	ESC();
	~ESC();
};
