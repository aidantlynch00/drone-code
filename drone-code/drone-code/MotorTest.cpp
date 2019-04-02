#include <iostream>
#include <stdint.h>
#include <wiringPi.h>
#include "Quadcopter.h"
#include "BerryIMU.h"
#include "ESC.h"
#include "KalmanFilter.h"
#include "PID.h"
#include "SoftPWM_GPIO.h"
#include "GPIO.h"

using namespace std;

#define BR 6
#define FR 13
#define FL 19
#define BL 26

#define RUD 0
#define AIL 1
#define ELE 2
#define THR 3

int main() {

	ESC esc = ESC{};
	RC rc = RC{};
	uint32_t* rc_values;

	while (true) {
		rc.read();
		rc_values = rc.getValues();

		esc.setPWM(BR, rc_values[ELE]);
		esc.setPWM(FR, rc_values[ELE]);
		esc.setPWM(FL, rc_values[ELE]);
		esc.setPWM(BL, rc_values[ELE]);
	}
}