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

#define BR 0
#define FR 1
#define FL 2
#define BL 3

#define RUD 0
#define AIL 1
#define ELE 2
#define THR 3
#define KILL 4

double constrain2(double value, double min, double max) {
	if(value < min) 
		return min;
	else if(value > max) 
		return max;
	else		       
		return value;
}

int main() {

	wiringPiSetupGpio();

	ESC* esc = new ESC();
	RC* rc = new RC();
	uint32_t* rc_values;
	uint32_t* rc_adj = new uint32_t[5];
	/*
	esc->setPWM(BR, 1000);
	esc->setPWM(FR, 1000);
	esc->setPWM(FL, 1000);
	esc->setPWM(BL, 1000);
	delay(5000);
	
	esc->setPWM(BR, 1200);
	esc->setPWM(FR, 1200);
	esc->setPWM(FL, 1200);
	esc->setPWM(BL, 1200);
	delay(3000);
	
	esc->setPWM(BR, 1999);
	esc->setPWM(FR, 1999);
	esc->setPWM(FL, 1999);
	esc->setPWM(BL, 1999);
	delay(3000);
	
	return 0;
	*/
	
	while (true) {
		rc->read();
		rc_values = rc->getValues();
		
		for (int channel = 0; channel < 5; channel++) {
			rc_adj[channel] = rc_values[channel];
			//rc_adj[channel] /= 50;
			//rc_adj[channel] *= 50;
			rc_adj[channel] = constrain2(rc_adj[channel], 1000, 1999);
		}
		
		if(rc_adj[KILL] > 1500){
			esc->setPWM(BR, 1000);
			esc->setPWM(FR, 1000);
			esc->setPWM(FL, 1000);
			esc->setPWM(BL, 1000);
		}
		else{
			esc->setPWM(BR, rc_adj[ELE]);
			esc->setPWM(FR, rc_adj[ELE]);
			esc->setPWM(FL, rc_adj[ELE]);
			esc->setPWM(BL, rc_adj[ELE]);
		}
		
		//cout << "BR: " << esc->getPWM(BR) << endl;
		//cout << "FR: " << esc->getPWM(FR) << endl;
		//cout << "FL: " << esc->getPWM(FL) << endl;
		//cout << "BL: " << esc->getPWM(BL) << endl;
		
	}
	
	delete esc;
	delete rc;
}
