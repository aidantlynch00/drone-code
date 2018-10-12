#include <wiringPi.h>
#include <iostream>
#include "GPIO.h"

GPIO::GPIO(int _bcm_pin = 17, int _mode = OUTPUT, int _pud = PUD_OFF) {
	bcm_pin = _bcm_pin;
	mode = _mode;
	pud = _pud;
	value = LOW; //off

	system("gpio export " + bcm_pin + (mode == OUTPUT) ? "out" : "in"); //might not work
	pinMode(bcm_pin, mode);
	pullUpDnControl(bcm_pin, pud);
}

GPIO::~GPIO() {
	system("gpio unexport " + bcm_pin); //same here
}

void GPIO::setMode(int _mode) {
	mode = _mode;
	pinMode(bcm_pin, mode);
}

void GPIO::setPUD(int _pud) {
	pud = _pud;
	pullUpDnControl(bcm_pin, pud);
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