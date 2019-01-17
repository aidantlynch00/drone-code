#include <wiringPi.h>
#include "GPIO.h"
#include "SoftPWM_GPIO.h"
#include "BerryIMU.h"
#include "math.h"
#include <iostream>
#include "PPMRead.h"
#include "KalmanFilter.h"


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
	PPMRead ppmRead17{ 7 };
	int x = 0;
	double* gyro_out;
	double* accel_out;
	double* mag_out;

	double startTime = 0;
	double endTime = 0;
	double dt = 0;

	while(x<1){
		dt = (endTime - startTime) / 1000000;
		system("clear");

		gyro_out = imu.readGyro();
		accel_out = imu.readAccel();
		mag_out = imu.readMag();

		double AccXangle = (float)(atan2(accel_out[1], accel_out[2]) + M_PI)*57.29578;
		double AccYangle = (float)(atan2(accel_out[2], accel_out[0]) + M_PI)*57.29578;

		AccXangle -= 180;
		if (AccYangle > 90)
			AccYangle -= 270;
		else
			AccYangle += 90;

		double rate_gyr_x = (float)gyro_out[0] * 0.5;
		double rate_gyr_y = (float)gyro_out[1] * 0.5;
		double rate_gyr_z = (float)gyro_out[2] * 0.5;

		KalmanFilter* filter = new KalmanFilter();
		AccXangle = filter->kalmanFilterX(AccXangle, rate_gyr_x, dt);
		AccYangle = filter->kalmanFilterY(AccYangle, rate_gyr_y, dt);

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
		
		cout << "Mag Raw X: " << mag_out[0] << endl << "Mag Raw Y: " << mag_out[1] << endl << "Mag Raw Z: " << mag_out[2] << endl << endl;

		if (endTime - startTime < 1999) {
			delayMicroseconds(1999 - (endTime - startTime));
		}
	}

	delete mag_out;
	delete gyro_out;
	delete accel_out;

	return 0;
}
