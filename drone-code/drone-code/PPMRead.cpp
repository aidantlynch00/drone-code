#include "PPMRead.h"
#include <wiringPi.h>

PPMRead::PPMRead()
{
	pinMode(RC_CHANNEL_1, INPUT);
	pinMode(RC_CHANNEL_1, INPUT);
	syncListener();
	
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
