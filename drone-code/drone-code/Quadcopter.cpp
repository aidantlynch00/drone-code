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
	//ya = 0;

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
	//cout << "Angle Z: " << ya << endl << endl;

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
	
	cout << "DT: " << dt * 1000000 << endl;
}



void Quadcopter::run() {
	bool flying = true;
	int count = 0;
	int buffer = 50;

	//Pitch is rotating about the Y axis, Roll is rotating about the X axis, Yaw is rotating about the Z axis

	while (flying) {

		//----Collect RC Target----\\
	
		rc_values = rc->getValues();

		for (int channel = 0; channel < 4; channel++) {
			//rc_adj[channel] = map_value(rc_values[channel], low, high, 1100, 1900);
			rc_adj[channel] = rc_values[channel];
			rc_adj[channel] /= buffer;
			rc_adj[channel] *= buffer;
			rc_adj[channel] = constrain(rc_adj[channel], 1000, 2000);
		}

		count = 0;

		while (true) {
			dt = (endTime - startTime) / 1000000.0;
			startTime = micros();
			count++;

			//Get values from accelerometer, gyroscope, and magnetometer
			accel_out = imu->readAccel();
			gyro_out = imu->readGyro();
			//mag_out = imu->readMag();

			//Gyro Calcs
			rv = (float)gyro_out[0] * 0.07; //rgx
			pv = (float)gyro_out[1] * 0.07; //rgy
			yv = (float)gyro_out[2] * 0.07; //rgz

			//Accel Calcs
			ra = (float)(atan2(accel_out[1], accel_out[2]) + M_PI)*57.29578;
			pa = (float)(atan2(accel_out[2], accel_out[0]) + M_PI)*57.29578;

			//Complementary Filter
			//ra = .98 * (ra + rv * dt) + .02 * accel_ra;
			//pa = .98 * (pa + pv * dt) + .02 * accel_pa;

			//Convert angles to +/- 180
			//ra -= 180;
			//if (pa > 90) pa -= 270;
			//else         pa += 90;
			if (ra > 180)		
				ra -= 360;

			//delete accel_out;
			//delete gyro_out;

			double ra_target = map_value(rc_adj[AIL], 1000, 2000, -33, 33);
			double pa_target = map_value(rc_adj[THR], 1000, 2000, -33, 33);
			double yv_target = map_value(rc_adj[RUD], 1000, 2000, -180, 180);
			double lift = constrain(rc_adj[ELE], 1100, 1900);

			//----------PID's----------\\
			
			if (lift > 1100) {
				ra_pid_out = ra_pid.compute(ra, ra_target, dt);
				pa_pid_out = pa_pid.compute(pa, pa_target, dt);
				yv_pid_out = ya_pid.compute(yv, yv_target, dt);
			}

			//------Change Speed-------\\

			motors["FL"]->setPWM(constrain(lift + ra_pid_out + pa_pid_out - yv_pid_out, 1100, 2000));
			motors["FR"]->setPWM(constrain(lift - ra_pid_out + pa_pid_out + yv_pid_out, 1100, 2000));
			motors["BL"]->setPWM(constrain(lift + ra_pid_out - pa_pid_out - yv_pid_out, 1100, 2000));
			motors["BR"]->setPWM(constrain(lift - ra_pid_out - pa_pid_out - yv_pid_out, 1100, 2000));

			print();

			//--Loop time corrections--\\

			if (micros() - startTime < 13000) {
				delayMicroseconds(13000 - (endTime - startTime + 30));
			}

			endTime = micros();
			
			if (count == 5)
				break;
		}
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
