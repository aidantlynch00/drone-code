#include "IMU_Test.h"
#include "LSM9DS1.h"
#include <wiringPi.h>
#include <wiringPiI2C.h>

IMU_Test::IMU_Test(int _accelID, int _gyroID) {
	accelID = _accelID;
	gyroID = _gyroID;
}

IMU_Test::~IMU_Test(){

}

double* IMU_Test::readAccelerometer() {
	double* accelValues = new double[3];
	*accelValues	   = (double)(wiringPiI2CReadReg8(accelID, LSM9DS1_OUT_X_L_XL) | wiringPiI2CReadReg8(accelID, LSM9DS1_OUT_X_H_XL) << 8);
	*(accelValues + 1) = (double)(wiringPiI2CReadReg8(accelID, LSM9DS1_OUT_Y_L_XL) | wiringPiI2CReadReg8(accelID, LSM9DS1_OUT_Y_H_XL) << 8);
	*(accelValues + 2) = (double)(wiringPiI2CReadReg8(accelID, LSM9DS1_OUT_Z_L_XL) | wiringPiI2CReadReg8(accelID, LSM9DS1_OUT_Z_H_XL) << 8);

	return accelValues;
}

double* IMU_Test::readGyroscope() {
	double* gyroValues = new double[3];
	*gyroValues		  = (double)(wiringPiI2CReadReg8(gyroID, LSM9DS1_OUT_X_L_G) | wiringPiI2CReadReg8(gyroID, LSM9DS1_OUT_X_H_G) << 8);
	*(gyroValues + 1) = (double)(wiringPiI2CReadReg8(gyroID, LSM9DS1_OUT_Y_L_G) | wiringPiI2CReadReg8(gyroID, LSM9DS1_OUT_Y_H_G) << 8);
	*(gyroValues + 2) = (double)(wiringPiI2CReadReg8(gyroID, LSM9DS1_OUT_Z_L_G) | wiringPiI2CReadReg8(gyroID, LSM9DS1_OUT_Z_H_G) << 8);

	return gyroValues;
}
