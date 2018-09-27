#include <wiringPi.h>
#include <softPwm.h>
#include "SoftPWM_GPIO.h"

SoftPWM_GPIO::SoftPWM_GPIO(int bcm_pin, int _duty_cycle = 100): GPIO(bcm_pin, OUTPUT, PUD_OFF){
	softPwmCreate(bcm_pin, 0, _duty_cycle);
	duty_cycle = _duty_cycle;
	value = 0;
}

SoftPWM_GPIO::~SoftPWM_GPIO(){
	//Base Class constructor implicity called here
}

void SoftPWM_GPIO::change_duty(int change) {
	duty_cycle += change;
}

int SoftPWM_GPIO::get_duty() {
	return duty_cycle;
}

void SoftPWM_GPIO::set_duty(int _duty_cycle) {
	duty_cycle = _duty_cycle;
}

void SoftPWM_GPIO::change_value(int change) {
	value += change;
}

void SoftPWM_GPIO::set_value(int _value) {
	value = _value;
}