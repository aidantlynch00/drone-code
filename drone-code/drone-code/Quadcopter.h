#pragma once
#include <map>
#include <string>
#include "ESC.h"
#include "BerryIMU.h"
#include "PID.h"
#include "KalmanFilter.h"
#include "RC.h"

using namespace std;

class Quadcopter {

private:
	const double declination = 43.92 / 1000.0;

	map<string, ESC*> motors;
	BerryIMU* imu;

	//VALUES NEEDED FOR CALCULATIONS
	//Angle variables
	double ra;
	double pa;
	double ya;

	//Angular velocity variables
	double rv;
	double pv;
	double yv;

	//Filter
	KalmanFilter* kalmanFilterX;
	KalmanFilter* kalmanFilterY;
	KalmanFilter* kalmanFilterZ;

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
	double* mag_out;

	//PID output arrays
	double* ra_pid_out;
	double* pa_pid_out;
	double* ya_pid_out;

	double* rv_pid_out;
	double* pv_pid_out;
	double* yv_pid_out;

	//Time variables
	double startTime;
	double endTime;
	double dt;


public:
	Quadcopter();
	~Quadcopter();
	void print();
	void run();
};
