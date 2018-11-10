#include <wiringPi.h>
#include "GPIO.h"
#include "SoftPWM_GPIO.h"
#include "BerryIMU.h"
#include <iostream>


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
	BerryIMU imu{0x6a, 0x1c};
	
	while(true){
		double* gyro_out = imu.readGyro();
		double* accel_out = imu.readAccel();
		
		cout << "GX: " << gyro_out[0] << endl;
		cout << "GY: " << gyro_out[1] << endl;
		cout << "GZ: " << gyro_out[2] << endl;
		cout << "AX: " << accel_out[0] << endl;
		cout << "AY: " << accel_out[1] << endl;
		cout << "AZ: " << accel_out[2] << endl << endl;
		
		delete gyro_out;
		delete accel_out;
		delay(2000);
	}

	return 0;
}
