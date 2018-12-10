#include <map>
#include <string>
#include "ESC.h"
#include "Quadcopter.h"
#include "PID.h"
#include "BerryIMU.h"

Quadcopter::Quadcopter() {
	//TODO: Replace pin numbers when hardware is connected
	motors["FL"] = new ESC(1);
	motors["FR"] = new ESC(1);
	motors["BL"] = new ESC(1);
	motors["BR"] = new ESC(1);

	imu = new BerryIMU{ 0x00, 0x00 };
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

	//X is forward/backward, Y is side to side, Z is up/down

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
	
	//Angle variables
	double rd = 0;
	double pd = 0;
	double yd = 0;

	//Angular velocity variables
	double rv = 0;
	double pv = 0;
	double yv = 0;

	//Angular acceleration variables
	double ra = 0;
	double pa = 0;
	double ya = 0;
	
	//Distance PIDs -Used for auto
	PID xd_pid{ 0, 0, 0 };
	PID yd_pid{ 0, 0, 0 };
	PID zd_pid{ 0, 0, 0 };

	//Velocity PIDs
	PID xv_pid{ 0, 0, 0 };
	PID yv_pid{ 0, 0, 0 };
	PID zv_pid{ 0, 0, 0 };

	//Roll, Pitch, Yaw PIDs
	PID rd_pid{ 0, 0, 0 };
	PID pd_pid{ 0, 0, 0 };
	PID yd_pid{ 0, 0, 0 };

	//Roll, Pitch, and Yaw angular velocity PID's
	PID rv_pid{ 0, 0, 0 };
	PID pv_pid{ 0, 0, 0 };
	PID yv_pid{ 0, 0, 0 };

	int startTime = 0;
	int endTime = 0;
	while (flying) {
		double dt = endTime - startTime;
		startTime = micros();

		//Get values from accel and gyro
		double* accelValues = imu->readAccel();
		double* gyroValues  = imu->readGyro();

		xa = accelValues[0];
		ya = accelValues[1];
		za = accelValues[2];

		ra = accelValues[0];
		pa = accelValues[1];
		ya = accelValues[2];

		delete accelValues;
		delete gyroValues;

		//-------Integration-------\\

		//Integrate Acceleration values to get velocity
		xv += xa * dt;
		yv += ya * dt;
		zv += za * dt;

		//Integrate again to get distance values
		xd += xv * dt;
		yd += yv * dt;
		zd += zv * dt;

		//Integrate angular accel values from gyro to get angular velocity
		rv += ra * dt;
		pv += pa * dt;
		yv += ya * dt;

		//Integrate again to get the angle values
		rd += rv * dt;
		pd += pv * dt;
		yd += yv * dt;

		//----Collect RC Target----\\
		
		double xv_target = 0;
		double yv_target = 0;
		double zv_target = 0;

		//----------PID's----------\\

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

		//------Change Speed-------\\
		
		double z_pwm = /*hover_pwm + */zv_out;

		//--Loop time corrections--\\
		endTime = micros();
		
		if (endTime - startTime < 1999) {
			delayMicroseconds(1999 - (endTime - startTime));
		}
	}
}