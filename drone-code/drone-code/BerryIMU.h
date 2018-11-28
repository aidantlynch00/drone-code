#pragma once
#include <wiringPi.h>
#include <wiringPiI2C.h>

class BerryIMU {

private:
	double loopTime;

public:
	BerryIMU(int accelReg, int gyroReg);
	~BerryIMU();
	double* readAccel(double *a); //   m/sec^2 in x, y, z
	double* readGyro(double *g); // angular acceleration
	double getLoopTime(); // milli sec
	void enableIMU();
	void writeAccReg(uint8_t reg, uint8_t value);
	void writeMagReg(uint8_t reg, uint8_t value);
	void writeGyrReg(uint8_t reg, uint8_t value);
	void selectDevice(int file, int addr);
	void  readBlock(uint8_t command, uint8_t size, uint8_t *data);
};
