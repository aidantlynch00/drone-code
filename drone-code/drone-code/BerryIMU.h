#pragma once
#include <wiringPi.h>
#include <wiringPiI2C.h>

class BerryIMU {

private:
	double loopTime;

public:
	BerryIMU(int accelReg, int gyroReg);
	double* readAccel();
	double* readGyro();
	double getLoopTime();
};
