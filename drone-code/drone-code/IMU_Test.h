#pragma once

class IMU_Test {

private:
	int accelID, gyroID;

public:
	IMU_Test(int accelID, int gyroID);
	~IMU_Test();
	double* readAccelerometer();
	double* readGyroscope();
};