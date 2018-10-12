#include <map>
#include <string>
#include "ESC.h"
#include "Quadcopter.h"

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