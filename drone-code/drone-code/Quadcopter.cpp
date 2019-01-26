#include <map>
#include <string>
#include <iostream>
#include "math.h"
#include "ESC.h"
#include "Quadcopter.h"
#include "PID.h"
#include "BerryIMU.h"
#include "KalmanFilter.h"
#include "RC.h"

using namespace std;

//Non-member function used for value mapping
double map_value(double value, double low1, double high1, double low2, double high2) {
	return low2 + (high2 - low2) * ((value - low1) / (high1 - low1));
}

Quadcopter::Quadcopter() {

	//TODO: Replace pin numbers when hardware is connected
	imu = new BerryIMU{};
	rc_adj = new uint16_t[3];

	ra = 0;
	pa = 0;
	ya = 0;

	rv = 0;
	pv = 0;
	yv = 0;

	kalmanFilterX = new KalmanFilter();
	kalmanFilterY = new KalmanFilter();
	kalmanFilterZ = new KalmanFilter();

	motors["FL"] = new ESC(1);
	motors["FR"] = new ESC(1);
	motors["BL"] = new ESC(1);
	motors["BR"] = new ESC(1);

	startTime = 0;
	endTime = 0;
}

Quadcopter::~Quadcopter() {
	delete imu;

	delete kalmanFilterX;
	delete kalmanFilterY;
	delete kalmanFilterZ;

	delete motors["FL"];
	delete motors["FR"];
	delete motors["BL"];
	delete motors["BR"];
	motors.clear();
}



void Quadcopter::print() {
	system("clear");

	cout << "Angle X: " << ra << endl;
	cout << "Angle Y: " << pa << endl;
	cout << "Angle Z: " << ya << endl << endl;

	cout << "Rate X: " << rv << endl;
	cout << "Rate Y: " << pv << endl;
	cout << "Rate Z: " << yv << endl << endl;
	
	cout << "RUD: " << rc_adj[0] << endl;
	cout << "AIL: " << rc_adj[1] << endl;
	cout << "ELE: " << rc_adj[2] << endl;
	cout << "THR: " << rc_adj[3] << endl << endl;
}



void Quadcopter::run() {
	bool flying = true;
	int buffer = 50;
	int low = 1000;
	int high = 2000;

	//Pitch is rotating about the Y axis, Roll is rotating about the X axis, Yaw is rotating about the Z axis
	
	while (flying) {
		dt = (endTime - startTime) / 1000000;
		startTime = micros();

		//TODO: Use magnotemeter to help with yaw

		//Get values from accelerometer, gyroscope, and magnetometer
		accel_out = imu->readAccel();
		gyro_out = imu->readGyro();
		mag_out = imu->readMag();

		//accel calcs
		ra = (float)(atan2(accel_out[1], accel_out[2]) + M_PI)*57.29578;
		pa = (float)(atan2(accel_out[2], accel_out[0]) + M_PI)*57.29578;

		//mag calcs
		double accXnorm = accel_out[0] / sqrt(accel_out[0] * accel_out[0] + accel_out[1] * accel_out[1] + accel_out[2] * accel_out[2]);
		double accYnorm = accel_out[1] / sqrt(accel_out[0] * accel_out[0] + accel_out[1] * accel_out[1] + accel_out[2] * accel_out[2]);

		double magXcomp = mag_out[0] * cos(asin(accXnorm)) + mag_out[2] * sin(pa);
		double magYcomp = mag_out[0] * sin(asin(accYnorm / cos(pa)))*sin(asin(accXnorm)) + mag_out[1] * cos(asin(accYnorm / cos(pa))) - mag_out[2] * sin(asin(accYnorm / cos(pa)))*cos(asin(accXnorm));

		magXcomp = *mag_out*cos(pa) + *(mag_out + 2)*sin(pa);
		magYcomp = *mag_out*sin(ra)*sin(pa) + *(mag_out + 1)*cos(ra) - *(mag_out + 2)*sin(ra)*cos(pa);

		ya = 180 * atan2(magYcomp, magXcomp) / M_PI;
		ya += declination * (180 / M_PI);

		if (ya > 360)
			ya -= 360;

		//gyro calcs
		rv = (float)gyro_out[0] * 0.5; //rgx
		pv = (float)gyro_out[1] * 0.5; //rgy
		yv = (float)gyro_out[2] * 0.5; //rgz

		//ra = kalmanFilterX->compute(ra, rv, dt);
		//pa = kalmanFilterY->compute(pa, pv, dt);
		//ya = kalmanFilterZ->compute(ya, yv, dt);

		//Convert angles to +/- 180
		             ra -= 180;
		if (pa > 90) pa -= 270;
		else         pa +=  90;

		delete accel_out;
		delete gyro_out;

		//----Collect RC Target----\\
	
		rc_values = rc->getValues();

		for (int channel = 0; channel < 4; channel++) {
			rc_adj[channel] = map_value(rc_values[channel], low, high, 1100, 1900);
			rc_adj[channel] /= buffer;
			rc_adj[channel] *= buffer;
		}

	

		double ra_target = 0;//map_value(rc_values[0], 1000, 2000, -45, 45);
		double pa_target = 0;//map_value(rc_values[0], 1000, 2000, -45, 45);;
		double ya_target = 0;//map_value(rc_values[0], 1000, 2000, -45, 45);;

		//----------PID's----------\\

		ra_pid_out = ra_pid.compute(ra, ra_target, dt);
		pa_pid_out = pa_pid.compute(pa, pa_target, dt);
		ya_pid_out = ya_pid.compute(ya, ya_target, dt);

		//rv_pid_out = rv_pid.compute(rv, rv_target, dt);
		//pv_pid_out = pv_pid.compute(pv, pv_target, dt);
		//yv_pid_out = yv_pid.compute(yv, yv_target, dt);

		//------Change Speed-------\\
		
		//double z_pwm = /*hover_pwm + */zv_out;


		print();


		//--Loop time corrections--\\
		
		endTime = micros();
		
		if (endTime - startTime < 80999) {
			delayMicroseconds(80999 - (endTime - startTime));
		}
	}
}































/*
//Target velocity
	double xv_target = 0;
	double yv_target = 0;
	double zv_target = 0;

	//Distance variables
	double xd = 0;
	double yd = 0;
	double zd = 0;

	//Velocity variables
	double xv = 0;
	double yv = 0;
	double zv = 0;

	//Acceleration variables
	double xa = 0;
	double ya = 0;
	double za = 0;


		//Distance PIDs -Used for auto
	PID xd_pid{ 0, 0, 0 };
	PID yd_pid{ 0, 0, 0 };
	PID zd_pid{ 0, 0, 0 };

	//Velocity PIDs
	PID xv_pid{ 0, 0, 0 };
	PID yv_pid{ 0, 0, 0 };
	PID zv_pid{ 0, 0, 0 };


	//Integrate Acceleration values to get velocity
		xv += xa * dt;
		yv += ya * dt;
		zv += za * dt;

		//Integrate again to get distance values
		xd += xv * dt;
		yd += yv * dt;
		zd += zv * dt;


		double xv_out;
		double yv_out;
		double zv_out;

		double* xv_out_arr;
		double* yv_out_arr;
		double* zv_out_arr;

		xv_out_arr = xv_pid.compute(xv, xv_target, dt);
		yv_out_arr = yv_pid.compute(yv, yv_target, dt);
		zv_out_arr = zv_pid.compute(zv, zv_target, dt);

		xv_out = xv_out_arr[0] + xv_out_arr[1] + xv_out_arr[2];
		yv_out = yv_out_arr[0] + yv_out_arr[1] + yv_out_arr[2];
		zv_out = zv_out_arr[0] + zv_out_arr[1] + zv_out_arr[2];

		delete xv_out_arr;
		delete yv_out_arr;
		delete zv_out_arr;


			//-------Integration-------\\



		//Integrate angular accel values from gyro to get angular velocity
		rv += ra * dt;
		pv += pa * dt;
		yv += ya * dt;

		//Integrate again to get the angle values
		ra += rv * dt;
		pd += pv * dt;
		yd += yv * dt;

*/
