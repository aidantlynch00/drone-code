#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdint.h>
#include <stdio.h>
#include "linux/i2c-dev.h"
#include "LSM9DS0.h"
#include "LSM9DS1.h"
#include "BerryIMU.h"

int file; 
int LSM9DS0 = 0;
int LSM9DS1 = 0;

BerryIMU::BerryIMU(int accelReg, int gyroReg){

}

BerryIMU::~BerryIMU(){

}

double * BerryIMU::readAccel(){
	return nullptr;
}

double * BerryIMU::readGyro() {
	return nullptr;
}

double BerryIMU::getLoopTime() {
	return 0.0;
}

void BerryIMU::writeAccReg(uint8_t reg, uint8_t value){

}

void BerryIMU::writeMagReg(uint8_t reg, uint8_t value){

}

void BerryIMU::writeGyrReg(uint8_t reg, uint8_t value){

}

void enableIMU() {
	if (LSM9DS0) {//For BerryIMUv1
		// Enable accelerometer.
		writeAccReg(LSM9DS0_CTRL_REG1_XM, 0b01100111); //  z,y,x axis enabled, continuous update,  100Hz data rate
		writeAccReg(LSM9DS0_CTRL_REG2_XM, 0b00100000); // +/- 16G full scale

		//Enable the magnetometer
		writeMagReg(LSM9DS0_CTRL_REG5_XM, 0b11110000); // Temp enable, M data rate = 50Hz
		writeMagReg(LSM9DS0_CTRL_REG6_XM, 0b01100000); // +/-12gauss
		writeMagReg(LSM9DS0_CTRL_REG7_XM, 0b00000000); // Continuous-conversion mode

		// Enable Gyro
		writeGyrReg(LSM9DS0_CTRL_REG1_G, 0b00001111); // Normal power mode, all axes enabled
		writeGyrReg(LSM9DS0_CTRL_REG4_G, 0b00110000); // Continuos update, 2000 dps full scale
	}
	if (LSM9DS1) {//For BerryIMUv2      
		// Enable the gyroscope
		writeGyrReg(LSM9DS1_CTRL_REG4, 0b00111000);      // z, y, x axis enabled for gyro
		writeGyrReg(LSM9DS1_CTRL_REG1_G, 0b10111000);    // Gyro ODR = 476Hz, 2000 dps
		writeGyrReg(LSM9DS1_ORIENT_CFG_G, 0b10111000);   // Swap orientation 

		// Enable the accelerometer
		writeAccReg(LSM9DS1_CTRL_REG5_XL, 0b00111000);   // z, y, x axis enabled for accelerometer
		writeAccReg(LSM9DS1_CTRL_REG6_XL, 0b00101000);   // +/- 16g

		//Enable the magnetometer
		writeMagReg(LSM9DS1_CTRL_REG1_M, 0b10011100);   // Temp compensation enabled,Low power mode mode,80Hz ODR
		writeMagReg(LSM9DS1_CTRL_REG2_M, 0b01000000);   // +/-12gauss
		writeMagReg(LSM9DS1_CTRL_REG3_M, 0b00000000);   // continuos update
		writeMagReg(LSM9DS1_CTRL_REG4_M, 0b00000000);   // lower power mode for Z axis
	}
}
