#include <wiringPi.h>
#include "GPIO.h"
#include "SoftPWM_GPIO.h"
#include "BerryIMU.h"
#include "math.h"
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
		
		double AccXangle = (float)(atan2(accel_out[1], accel_out[2]) + M_PI)*57.29578;
		double AccYangle = (float)(atan2(accel_out[2], accel_out[0]) + M_PI)*57.29578;

		double rate_gyr_x = (float)gyro_out[0] * 0.5;
		double rate_gyr_y = (float)gyro_out[1] * 0.5;
		double rate_gyr_z = (float)gyro_out[2] * 0.5;

		cout << "GX: " << gyro_out[0] << endl;
		cout << "GY: " << gyro_out[1] << endl;
		cout << "GZ: " << gyro_out[2] << endl;
		cout << "AX: " << accel_out[0] << endl;
		cout << "AY: " << accel_out[1] << endl;
		cout << "AZ: " << accel_out[2] << endl << endl;

		cout << "AngleX: " << AccXangle << endl;
		cout << "AngleY: " << AccYangle << endl << endl;
		cout << "RateX: " << rate_gyr_x << endl;
		cout << "RateY: " << rate_gyr_y << endl;
		cout << "RateZ: " << rate_gyr_z << endl << endl;
		
		delete gyro_out;
		delete accel_out;
		delay(500);
	}

	return 0;
}
