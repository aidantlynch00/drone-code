#include <wiringPi.h>
#include "GPIO.h"
#include "SoftPWM_GPIO.h"
#include "BerryIMU.h"
#include "math.h"
#include <iostream>
#include "KalmanFilter.h"
#include "RC.h"
#include "Quadcopter.h"


//link real-time library during execution with -lrt

// LED Pin - wiringPi pin 0 is BCM_GPIO 17.
// we have to use BCM numbering when initializing with wiringPiSetupSys
// when choosing a different pin number please use the BCM numbering, also
// update the Property Pages - Build Events - Remote Post-Build Event command 
// which uses gpio export for setup for wiringPiSetupSys

using namespace std;
int main(void)
{
	wiringPiSetupGpio();
	piHiPri(99);

	int rc = piThreadCreate(RC);
	if (rc != 0)
		printf("RC FAIL");

	int quad = piThreadCreate(QUAD);
	if (quad != 0)
		printf("QUAD FAIL");

	return 0;
}

PI_THREAD(RC)
{
	Quadcopter* quad = new Quadcopter();
	quad->run();
	delete quad;
}

PI_THREAD(QUAD)
{
	RC_Code* rc = new RC_Code();
	rc->read();
	delete rc;
}
