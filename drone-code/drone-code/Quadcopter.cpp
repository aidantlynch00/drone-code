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

#define RUD 0
#define AIL 1
#define ELE 2
#define THR 3

using namespace std;

double map_value(double value, double low1, double high1, double low2, double high2){
	return low2 + (high2 - low2) * (( value - low1) / (high1 - low1));
}

double constrain(double value, double min, double max) {
	if(value < min) 
		return min;
	else if(value > max) 
		return max;
	else		       
		return value;
}

Quadcopter::Quadcopter() {
	//TODO: Replace pin numbers when hardware is connected
	imu = new BerryIMU{};
	rc_adj = new uint32_t[3];

	ra = 0;
	pa = 0;

	accel_ra = 0;
	accel_pa = 0;

	yv = 0;

	rv = 0;
	pv = 0;

	kalmanFilterX = new KalmanFilter();
	kalmanFilterY = new KalmanFilter();
	kalmanFilterZ = new KalmanFilter();

	motors["FL"] = new ESC(6);
	motors["FR"] = new ESC(13);
	motors["BL"] = new ESC(19);
	motors["BR"] = new ESC(26);
	
	motors["FL"]->setPWM(1000);
	motors["FR"]->setPWM(1000);
	motors["BL"]->setPWM(1000);
	motors["BR"]->setPWM(1000);

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
	
	cout << "Angle X: " << ra << endl;
	cout << "Angle Y: " << pa << endl;

	cout << "Rate X: " << rv << endl;
	cout << "Rate Y: " << pv << endl;
	cout << "Rate Z: " << yv << endl << endl;
	
	cout << "RUD: " << rc_adj[RUD] << endl;
	cout << "AIL: " << rc_adj[AIL] << endl;
	cout << "ELE: " << rc_adj[ELE] << endl;
	cout << "THR: " << rc_adj[THR] << endl << endl;
	
	cout << "FL: " << motors["FL"]->getPWM() << endl;
	cout << "FR: " << motors["FR"]->getPWM() << endl;
	cout << "BL: " << motors["BL"]->getPWM() << endl;
	cout << "BR: " << motors["BR"]->getPWM() << endl << endl;
	
	cout << "DT: " << dt * 1000000 << endl << endl << endl << endl << endl << endl;
}



void Quadcopter::run() {
	bool flying = true;
	bool firstLoop = true;
	int count = 0;
	int buffer = 50;

	//Pitch is rotating about the Y axis, Roll is rotating about the X axis, Yaw is rotating about the Z axis

	while (flying) {

		dt = (endTime - startTime) / 1000000.0;
		startTime = micros();
		count++;

		//RC collection logic
		if (count == 16) {
			rc_values = rc->getValues();

			for (int channel = 0; channel < 4; channel++) {
				rc_adj[channel] = rc_values[channel];
				rc_adj[channel] /= buffer;
				rc_adj[channel] *= buffer;
				rc_adj[channel] = constrain(rc_adj[channel], 1000, 2000);
			}

			count = 0;
			print();
		}

		//Get values from accelerometer, gyroscope, and magnetometer
		accel_out = imu->readAccel();
		gyro_out = imu->readGyro();

		//Gyro Calcs
		rv = (float)gyro_out[0] * 0.07; //rgx
		pv = (float)gyro_out[1] * 0.07; //rgy
		yv = (float)gyro_out[2] * 0.07; //rgz

		//Accel Calcs
		accel_ra = (float)(atan2(accel_out[1], accel_out[2]) + M_PI)*57.29578;
		accel_pa = (float)(atan2(accel_out[2], accel_out[0]) + M_PI)*57.29578;
		
		if(firstLoop){
			ra = accel_ra; 
			pa = accel_pa;
			firstLoop = false;
		}
		
		cout << pa << endl;
		
		//Complementary Filter: TODO
		ra = .98 * (ra + (rv * dt)) + .02 * accel_ra;
		pa = .98 * (pa + (pv * dt)) + .02 * accel_pa;
		
		//Wrap around ALL OF THIS CODE IS FLAWED
		if(ra > 360)
			ra -= 360;
		if(pa > 360)
			pa -= 360;
		
		//Adjust to +/- 180
		if (ra > 180)		
			ra -= 360;
		if(pa >= 270)
			pa -= 450;
		else;
			//pa -= 90;

		double ra_target = map_value(rc_adj[AIL], 1000, 2000, -33, 33);
		double pa_target = map_value(rc_adj[THR], 1000, 2000, -33, 33);
		double yv_target = map_value(rc_adj[RUD], 1000, 2000, -180, 180);
		double lift = constrain(rc_adj[ELE], 1100, 1900);

		//----------PID's----------\\
			
		ra_pid_out = ra_pid.compute(ra, ra_target, dt);
		pa_pid_out = pa_pid.compute(pa, pa_target, dt);
		yv_pid_out = yv_pid.compute(yv, yv_target, dt);

		//------Change Speed-------\\
		
		int fl = lift + ra_pid_out + pa_pid_out - yv_pid_out;
		int fr = lift - ra_pid_out + pa_pid_out + yv_pid_out;
		int bl = lift + ra_pid_out - pa_pid_out + yv_pid_out;
		int br = lift - ra_pid_out - pa_pid_out - yv_pid_out;
		
		//fl = constrain(fl, 1000, 2000);
		//fr = constrain(fr, 1000, 2000);
		//bl = constrain(bl, 1000, 2000);
		//br = constrain(br, 1000, 2000);

		motors["FL"]->setPWM(fl);
		motors["FR"]->setPWM(fr);
		motors["BL"]->setPWM(bl);
		motors["BR"]->setPWM(br);

		//--Loop time corrections--\\
		
		endTime = micros();

		if (endTime - startTime < 4000) {
			delayMicroseconds(4000 - (endTime - startTime));
		}

		endTime = micros();
	}
}


/*mag calcs
double accXnorm = accel_out[0] / sqrt(accel_out[0] * accel_out[0] + accel_out[1] * accel_out[1] + accel_out[2] * accel_out[2]);
double accYnorm = accel_out[1] / sqrt(accel_out[0] * accel_out[0] + accel_out[1] * accel_out[1] + accel_out[2] * accel_out[2]);

double magXcomp = mag_out[0] * cos(asin(accXnorm)) + mag_out[2] * sin(pa);
double magYcomp = mag_out[0] * sin(asin(accYnorm / cos(pa)))*sin(asin(accXnorm)) + mag_out[1] * cos(asin(accYnorm / cos(pa))) - mag_out[2] * sin(asin(accYnorm / cos(pa)))*cos(asin(accXnorm));

magXcomp = *mag_out*cos(pa) + *(mag_out + 2)*sin(pa);
magYcomp = *mag_out*sin(ra)*sin(pa) + *(mag_out + 1)*cos(ra) - *(mag_out + 2)*sin(ra)*cos(pa);

ya = 180 * atan2(magYcomp, magXcomp) / M_PI;
ya += declination * (180 / M_PI);*/

//ra = kalmanFilterX->compute(ra, rv, dt);
//pa = kalmanFilterY->compute(pa, pv, dt);
//ya = kalmanFilterZ->compute(ya, yv, dt);

//rv_pid_out = rv_pid.compute(rv, rv_target, dt);
//pv_pid_out = pv_pid.compute(pv, pv_target, dt);
//yv_pid_out = yv_pid.compute(yv, yv_target, dt);

/*Possibly add in Median filter code here
double ra_sum = 0;
double pa_sum = 0;
for (int i = 0; i < 16; i++) {
	ra_sum += ra_mean[i];
	pa_sum += pa_mean[i];				
}

smooth_ra = ra_sum / 16;
smooth_pa = pa_sum / 16;
*/































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
