#include <wiringPi.h>
#include "GPIO.h"

// LED Pin - wiringPi pin 0 is BCM_GPIO 17.
// we have to use BCM numbering when initializing with wiringPiSetupSys
// when choosing a different pin number please use the BCM numbering, also
// update the Property Pages - Build Events - Remote Post-Build Event command 
// which uses gpio export for setup for wiringPiSetupSys
#define	LED	17

int main(void)
{
	wiringPiSetupSys();

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

	return 0;
}

//Notes: Soft PVM (pulse width modulation through software) is not suitable for motor controlling
//more Hard PVM pins in a controller board
//power might become an issue on control boards