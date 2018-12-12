#pragma once

class IMU_Test {

private:
	int accelID, gyroID;
	int file_i2c;
	int length;
	unsigned char buffer[60] = {0};

public:
	IMU_Test(int accelID, int gyroID);
	~IMU_Test();
	double* readAccelerometer();
	double* readGyroscope();
};