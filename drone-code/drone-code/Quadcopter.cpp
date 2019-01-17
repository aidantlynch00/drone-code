#include <map>
#include <string>
#include "math.h"
#include "ESC.h"
#include "Quadcopter.h"
#include "PID.h"
#include "BerryIMU.h"
#include "KalmanFilter.h"

Quadcopter::Quadcopter() {
	//TODO: Replace pin numbers when hardware is connected
	motors["FL"] = new ESC(1);
	motors["FR"] = new ESC(1);
	motors["BL"] = new ESC(1);
	motors["BR"] = new ESC(1);

	imu = new BerryIMU{};
}

Quadcopter::~Quadcopter() {
	delete motors["FL"];
	delete motors["FR"];
	delete motors["BL"];
	delete motors["BR"];
	motors.clear();

	delete imu;
}

void Quadcopter::run() {
	bool flying = true;

	//Pitch is rotating about the Y axis, Roll is rotating about the X axis, Yaw is rotating about the Z axis
	
	//Angle variables
	double ra = 0;
	double pa = 0;
	double ya = 0;

	//Angular velocity variables
	double rv = 0;
	double pv = 0;
	double yv = 0;

	//Filter
	KalmanFilter* filter = new KalmanFilter();

	//Roll, Pitch, Yaw PIDs
	PID ra_pid{ 0, 0, 0 };
	PID pa_pid{ 0, 0, 0 };
	PID ya_pid{ 0, 0, 0 };

	//Roll, Pitch, and Yaw angular velocity PID's
	PID rv_pid{ 0, 0, 0 };
	PID pv_pid{ 0, 0, 0 };
	PID yv_pid{ 0, 0, 0 };

	//Raw output arrays
	double* accel_out;
	double* gyro_out;

	//PID output arrays
	double* ra_pid_out;
	double* pa_pid_out;
	double* ya_pid_out;

	double* rv_pid_out;
	double* pv_pid_out;
	double* yv_pid_out;
	
	//Time variables
	double startTime = 0;
	double endTime = 0;
	double dt;

	while (flying) {
		dt = (endTime - startTime) / 1000000;
		startTime = micros();

		//TODO: Use magnotemeter to help with yaw

		//Get values from accel and gyro
		accel_out = imu->readAccel();
		gyro_out = imu->readGyro();

		ra = (float)(atan2(accel_out[1], accel_out[2]) + M_PI)*57.29578;
		pa = (float)(atan2(accel_out[2], accel_out[0]) + M_PI)*57.29578;
		//ya = ;

		rv = (float)gyro_out[0] * 0.5; //rgx
		pv = (float)gyro_out[1] * 0.5; //rgy
		yv = (float)gyro_out[2] * 0.5; //rgz

		ra = filter->kalmanFilterX(ra, rv, dt);
		pa = filter->kalmanFilterY(pa, pv, dt);
		//Kalman Filter Z??

		//Convert angles to +/- 180
		             ra -= 180;
		if (pa > 90) pa -= 270;
		else         pa +=  90;

		delete accel_out;
		delete gyro_out;

		//----Collect RC Target----\\
		
		double ra_target = 0;
		double pa_target = 0;
		double ya_target = 0;

		//----------PID's----------\\

		ra_pid_out = ra_pid.compute(ra, ra_target, dt);
		pa_pid_out = pa_pid.compute(pa, pa_target, dt);
		ya_pid_out = ya_pid.compute(ya, ya_target, dt);

		rv_pid_out = rv_pid.compute(rv, rv_target, dt);
		pv_pid_out = pv_pid.compute(pv, pv_target, dt);
		yv_pid_out = yv_pid.compute(yv, yv_target, dt);

		//------Change Speed-------\\
		
		double z_pwm = /*hover_pwm + */zv_out;

		//--Loop time corrections--\\
		endTime = micros();
		
		if (endTime - startTime < 1999) {
			delayMicroseconds(1999 - (endTime - startTime));
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