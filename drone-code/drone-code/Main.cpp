#include <wiringPi.h>
#include "GPIO.h"
#include "SoftPWM_GPIO.h"

// LED Pin - wiringPi pin 0 is BCM_GPIO 17.
// we have to use BCM numbering when initializing with wiringPiSetupSys
// when choosing a different pin number please use the BCM numbering, also
// update the Property Pages - Build Events - Remote Post-Build Event command 
// which uses gpio export for setup for wiringPiSetupSys

int main(void)
{
	wiringPiSetup();

	/*
	pinMode(LED, OUTPUT);

	while (true)
	{
		digitalWrite(LED, HIGH);  // On
		delay(500); // ms
		digitalWrite(LED, LOW);	  // Off
		delay(500);
	}*/

	//THIS IS HOW BLINK WOULD BE REWRITTEN USING GPIO CLASS
	
	GPIO* led = new GPIO(24, OUTPUT, PUD_OFF);
	led->set_value(HIGH);
	delay(1000);
	led->set_value(LOW);

	delete led;
	led = nullptr;

	//BLINK AT 50%
	SoftPWM_GPIO* pwm_led = new SoftPWM_GPIO(27);
	pwm_led->set_value(50);
	delay(1000);
	pwm_led->set_value(0);

	delete pwm_led;
	pwm_led = nullptr;

	return 0;
}