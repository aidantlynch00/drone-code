#include <wiringPi.h>
#include <iostream>
#include "GPIO.h"

GPIO::GPIO(int _bcm_pin, int _mode) {
	bcm_pin = _bcm_pin;
	mode = _mode;
	value = LOW; //off

	pinMode(bcm_pin, mode);
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
	if(mode == INPUT)
		value = digitalRead(bcm_pin);
		
	return value;
}

int GPIO::getPin() {
	return bcm_pin;
}
