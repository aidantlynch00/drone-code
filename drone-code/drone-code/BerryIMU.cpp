#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdint.h>
#include <stdio.h>
#include "linux/i2c-dev.h"
#include "fcntl.h"
#include "LSM9DS0.h"
#include "LSM9DS1.h"
#include "BerryIMU.h"

int file; 
int LSM9DS0 = 0;
int LSM9DS1 = 0;

BerryIMU::BerryIMU(int accelReg, int gyroReg){
	detectIMU();
	enableIMU();
}

BerryIMU::~BerryIMU(){
	
}

double * BerryIMU::readAccel(){
	uint8_t block[6];
	if (LSM9DS1) {
		selectDevice(file, LSM9DS1_ACC_ADDRESS);
		readBlock(0x80 | LSM9DS1_OUT_X_L_XL, sizeof(block), block);
	}
	double *a = new double[3];
	*a = (int16_t)(block[0] | block[1] << 8);
	*(a+1) = (int16_t)(block[2] | block[3] << 8);
	*(a+2) = (int16_t)(block[4] | block[5] << 8);
	return a;
}

double * BerryIMU::readGyro() {
	uint8_t block[6];
	if (LSM9DS1) {
		selectDevice(file, LSM9DS1_GYR_ADDRESS);
		readBlock(0x80 | LSM9DS1_OUT_X_L_G, sizeof(block), block);
	}
	double *g = new double[3];
	*g = (int16_t)(block[0] | block[1] << 8);
	*(g + 1) = (int16_t)(block[2] | block[3] << 8);
	*(g + 2) = (int16_t)(block[4] | block[5] << 8);
	return g;
}

void BerryIMU::writeAccReg(uint8_t reg, uint8_t value){
	if (LSM9DS1)
		selectDevice(file, LSM9DS1_ACC_ADDRESS);

	int result = i2c_smbus_write_byte_data(file, reg, value);
	if (result == -1) {
		printf("Failed to write byte to I2C Acc.");
	}
}

void BerryIMU::writeGyrReg(uint8_t reg, uint8_t value){
	if (LSM9DS1)
		selectDevice(file, LSM9DS1_MAG_ADDRESS);

	int result = i2c_smbus_write_byte_data(file, reg, value);
	if (result == -1) {
		printf("Failed to write byte to I2C Mag.");
	}
}

void BerryIMU::selectDevice(int file, int addr){
	if (ioctl(file, I2C_SLAVE, addr) < 0) {
		printf("Failed to select I2C device.");
	}
}

void BerryIMU::readBlock(uint8_t command, uint8_t size, uint8_t * data){
	int result = i2c_smbus_read_i2c_block_data(file, command, size, data);
	if (result != size) {
		printf("Failed to read block from I2C.");
	}
}

void BerryIMU::detectIMU()
{
	__u16 block[I2C_SMBUS_BLOCK_MAX];

	int res, bus, size;

	char filename[20];
	sprintf(filename, "/dev/i2c-%d", 1);
	file = open(filename, O_RDWR);
	if (file < 0) {
		printf("Unable to open I2C bus!");
	}

	//Detect if BerryIMUv2 (Which uses a LSM9DS1) is connected
	selectDevice(file, LSM9DS1_MAG_ADDRESS);
	int LSM9DS1_WHO_M_response = i2c_smbus_read_byte_data(file, LSM9DS1_WHO_AM_I_M);
	selectDevice(file, LSM9DS1_GYR_ADDRESS);
	int LSM9DS1_WHO_XG_response = i2c_smbus_read_byte_data(file, LSM9DS1_WHO_AM_I_XG);

	if (LSM9DS1_WHO_XG_response == 0x68 && LSM9DS1_WHO_M_response == 0x3d) {
		printf("\n\n\n#####   BerryIMUv2/LSM9DS1  DETECTED    #####\n\n");
		LSM9DS1 = 1;
	}
	if (!LSM9DS0 && !LSM9DS1) {
		printf("NO IMU DETECTED\n");
	}
}

void BerryIMU::enableIMU() {
	if (LSM9DS1) {//For BerryIMUv2      
		// Enable the gyroscope
		writeGyrReg(LSM9DS1_CTRL_REG4, 0b00111000);      // z, y, x axis enabled for gyro
		writeGyrReg(LSM9DS1_CTRL_REG1_G, 0b10111000);    // Gyro ODR = 476Hz, 2000 dps
		writeGyrReg(LSM9DS1_ORIENT_CFG_G, 0b10111000);   // Swap orientation 

		// Enable the accelerometer
		writeAccReg(LSM9DS1_CTRL_REG5_XL, 0b00111000);   // z, y, x axis enabled for accelerometer
		writeAccReg(LSM9DS1_CTRL_REG6_XL, 0b00101000);   // +/- 16g

	}
}