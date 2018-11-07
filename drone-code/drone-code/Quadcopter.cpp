#include <map>
#include <string>
#include "ESC.h"
#include "Quadcopter.h"
#include "PID.h"

Quadcopter::Quadcopter() {
	//TODO: Replace pin numbers when hardware is connected
	motors["FL"] = new ESC(1);
	motors["FR"] = new ESC(1);
	motors["BL"] = new ESC(1);
	motors["BR"] = new ESC(1);
}

Quadcopter::~Quadcopter() {
	delete motors["FL"];
	delete motors["FR"];
	delete motors["BL"];
	delete motors["BR"];
	motors.clear();
}

void Quadcopter::run() {
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
	PID xr_pid{ 0, 0, 0 };
	PID yp_pid{ 0, 0, 0 };
	PID zy_pid{ 0, 0, 0 };
}