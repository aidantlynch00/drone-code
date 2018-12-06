#include "PPMRead.h"
#include <wiringPi.h>

PPMRead::PPMRead()
{
	pinMode(RC_CHANNEL_1, INPUT);
	pinMode(RC_CHANNEL_1, INPUT);
	startTime = micros();
	currentTime = startTime;
	positionTime = startTime;
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
	return 0;
}

PPMRead::~PPMRead()
{
}
