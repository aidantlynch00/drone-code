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

void GPIO::set_mode(int _mode) {
	mode = _mode;
	pinMode(bcm_pin, mode);
}

void GPIO::set_pud(int _pud) {
	pud = _pud;
	pullUpDnControl(bcm_pin, pud);
}

void GPIO::set_value(int _value) {
	value = _value;
	digitalWrite(bcm_pin, value);
}

int GPIO::get_value() {
	return value;
}

int GPIO::get_pin() {
	return bcm_pin;
}