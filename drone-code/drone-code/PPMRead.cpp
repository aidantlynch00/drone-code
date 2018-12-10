#include "PPMRead.h"
#include <wiringPi.h>

PPMRead::PPMRead()
{
	pinMode(RC_CHANNEL_1, INPUT);
	pinMode(RC_CHANNEL_1, INPUT);
	cycle = 1;
	startTime = micros();
	currentTime = startTime;
	positionTime = currentTime - startTime;
	currentState = LOW;
	if (syncListener()) {
		valueReader(RC_CHANNEL_1);
	}
}

bool PPMRead::syncListener()
{
	bool signaRecieved = false;

	while (!signaRecieved) {
		
	}

	return false;
}

int PPMRead::valueReader(int pin)
{
	if (digitalRead(pin) == HIGH) {
		positionTime = micros() - startTime;
	}

	return 0;
}

PPMRead::~PPMRead()
{
}
