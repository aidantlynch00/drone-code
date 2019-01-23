#pragma once
#include <map>
#include <string>
#include "ESC.h"
#include "BerryIMU.h"

using namespace std;

class Quadcopter {

private:
	map<string, ESC*> motors;
	BerryIMU* imu;

	//VALUES NEEDED FOR CALCULATIONS
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
	double* mag_out;

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


public:
	Quadcopter();
	~Quadcopter();
	void print();
	void run();
};