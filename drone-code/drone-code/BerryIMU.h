#pragma once
#include <wiringPi.h>
#include <wiringPiI2C.h>

class BerryIMU {

private:
	double loopTime;

public:
	BerryIMU(int accelReg, int gyroReg);
	~BerryIMU();
	double* readAccel(); //   m/sec^2
	double* readGyro(); // angular acceleration
	double getLoopTime(); // sec
	void writeAccReg(uint8_t reg, uint8_t value);
	void writeMagReg(uint8_t reg, uint8_t value);
	void writeGyrReg(uint8_t reg, uint8_t value);
};
