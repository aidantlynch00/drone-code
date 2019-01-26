#include <wiringPi.h>
#include "GPIO.h"
#include "SoftPWM_GPIO.h"
#include "BerryIMU.h"
#include "math.h"
#include <iostream>
#include "KalmanFilter.h"
#include "RC_Code.h"
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
	BerryIMU imu{};

	Quadcopter* quad = new Quadcopter();
	quad->run();
	
	delete quad;

	return 0;
}
