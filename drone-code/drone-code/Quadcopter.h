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

public:
	Quadcopter();
	~Quadcopter();
	void run();
};