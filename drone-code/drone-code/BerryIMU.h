#pragma once
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdint.h>

class BerryIMU {

private:
	double loopTime;

public:
	BerryIMU();
	~BerryIMU();
	double* readAccel(); //   m/sec^2 in x, y, z
	double* readGyro(); // angular acceleration
	void enableIMU();
	void detectIMU();
	void writeAccelReg(uint8_t reg, uint8_t value);
	void writeGyroReg(uint8_t reg, uint8_t value);
	void selectDevice(int file, int addr);
	void  readBlock(uint8_t command, uint8_t size, uint8_t *data);
};
