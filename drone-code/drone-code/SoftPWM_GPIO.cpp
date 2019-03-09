#include <wiringPi.h>
#include "softPwm_wpi.h"
#include "SoftPWM_GPIO.h"
#include <iostream>

using namespace std;

SoftPWM_GPIO::SoftPWM_GPIO(){
	value = 0;
}

SoftPWM_GPIO::SoftPWM_GPIO(int bcm_pin){
	pin = bcm_pin;
	pinMode(bcm_pin, OUTPUT);
	softPwmCreate(bcm_pin, 1000, 2000);
	value = 1000;
}

SoftPWM_GPIO::~SoftPWM_GPIO(){
	softPwmStop(pin);
}

void SoftPWM_GPIO::setValue(uint32_t _value) {
	value = _value;
	//cout << "Pin: " << pin << "   Value: " << value << endl;
	softPwmWrite(pin, value);
}

uint32_t SoftPWM_GPIO::getValue() {
	return value;
}
