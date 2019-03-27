#include <wiringPi.h>
#include <iostream>
#include "GPIO.h"

GPIO::GPIO(int _bcm_pin, int _mode) {
	bcm_pin = _bcm_pin;
	mode = _mode;
	pud = _pud;
	value = LOW; //off

	pinMode(bcm_pin, mode);
	pullUpDnControl(bcm_pin, PUD_OFF);
}

GPIO::~GPIO() {

}

void GPIO::setMode(int _mode) {
	mode = _mode;
	pinMode(bcm_pin, mode);
}

void GPIO::setValue(int _value) {
	value = _value;
	digitalWrite(bcm_pin, value);
}

int GPIO::getValue() {
	return value;
}

int GPIO::getPin() {
	return bcm_pin;
}