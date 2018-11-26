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
	


	//Distance PIDs
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

	while (flying) {

		//Get values from accel and gyro
		double* accelValues = imu->readAccel();
		double* gyroValues  = imu->readGyro();

	}
}