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
	//Distance PIDs
	PID xd_pid{ 0, 0, 0 };
	PID yd_pid{ 0, 0, 0 };
	PID zd_pid{ 0, 0, 0 };

	//Velocity PIDs
	PID xv_pid{ 0, 0, 0 };
	PID yv_pid{ 0, 0, 0 };
	PID zv_pid{ 0, 0, 0 };

	//Roll, Pitch, Yaw PIDs
	PID ra_pid{ 0, 0, 0 };
	PID pa_pid{ 0, 0, 0 };
	PID ya_pid{ 0, 0, 0 };

	//Roll, Pitch, and Yaw angular velocity PID's
	PID rv_pid{ 0, 0, 0 };
	PID pv_pid{ 0, 0, 0 };
	PID yv_pid{ 0, 0, 0 };

	while (flying) {

		//Get values from accel and gyro
		double* accelValues = imu->readAccel();

	}
}