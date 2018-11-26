#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdint.h>
#include <stdio.h>
#include "linux/i2c-dev.h"
#include "LSM9DS0.h"
#include "LSM9DS1.h"

int file; 
int LSM9DS0 = 0;
int LSM9DS1 = 0;