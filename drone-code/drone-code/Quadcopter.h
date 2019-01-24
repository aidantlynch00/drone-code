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

	//Roll, Pitch, Yaw PIDs
	PID ra_pid;
	PID pa_pid;
	PID ya_pid;

	//Roll, Pitch, and Yaw angular velocity PID's
	PID rv_pid;
	PID pv_pid;
	PID yv_pid;

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