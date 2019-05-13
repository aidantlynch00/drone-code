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

double map2(double value, double low1, double high1, double low2, double high2){
	return low2 + (high2 - low2) * (( value - low1) / (high1 - low1));
}

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
	
	int count = 0;
	while (true) {
		count++;
		
		if(count == 16){
			rc->read();
			rc_values = rc->getValues();
		
			for (int channel = 0; channel < 5; channel++) {
				rc_adj[channel] = rc_values[channel];
				//rc_adj[channel] /= 25;
				//rc_adj[channel] *= 25;
				rc_adj[channel] = constrain2(rc_adj[channel], 1000, 1999);
			}
			
			count = 0;
		}
		
		int ra = map2(rc_adj[AIL], 1000, 2000, -100, 100);
		int pa = map2(rc_adj[THR], 1000, 2000, -100, 100);
		int ya = map2(rc_adj[RUD], 1000, 2000, -100, 100);
		int lift = constrain2(rc_adj[ELE], 1000, 1800);
		
		int fl = lift + ra - pa - ya;
		int fr = lift - ra - pa + ya;
		int bl = lift + ra + pa + ya;
		int br = lift - ra + pa - ya;
		
		if(rc_adj[KILL] > 1500){
			esc->setPWM(BR, 1000);
			esc->setPWM(FR, 1000);
			esc->setPWM(FL, 1000);
			esc->setPWM(BL, 1000);
		}
		else{
			esc->setPWM(BR, br);
			esc->setPWM(FR, fr);
			esc->setPWM(FL, fl);
			esc->setPWM(BL, bl);
		}
		
		cout << "BR: " << esc->getPWM(BR);
		cout << "   FR: " << esc->getPWM(FR) << endl;
		cout << "FL: " << esc->getPWM(FL);
		cout << "   BL: " << esc->getPWM(BL) << endl << endl;
		
	}
	
	delete esc;
	delete rc;
}
