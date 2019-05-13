#include "PID.h"
#include <iostream>

using namespace std;

PID::PID(double _kp, double _ki, double _kd) {
	kp = _kp;
	ki = _ki;
	kd = _kd;

	previous_error = 0;
	i_error = 0;
}

double PID::error(double input, double target) {
	return (target - input);
}

double PID::compute(double input, double target, double dt) {
	//cout << "Input: " << input << endl;
	//cout << "Target: " << target << endl;
	//cout << "DT: " << dt << endl;
	double err = error(input, target);
	//cout << "Error: " << err << endl << endl;
	double p_error = err; ///Pointer weirdness????
	//cout << "P Error: " << p_error << endl;
	//i_error += .5 * (err + previous_error) * dt; 
	i_error += err * dt;
	//cout << "I Error: " << i_error << endl;
	double d_error = (err - previous_error) / dt;
	//cout << "D Error: " << d_error << endl << endl;

	double p_output = kp * p_error;
	double i_output = ki * i_error;
	double d_output = kd * d_error;
	
	//cout << "P Output: " << p_output << endl;
	//cout << "I Output: " << i_output << endl;
	//cout << "D Output: " << d_output << endl << endl;

	previous_error = err;
	double output = p_output + i_output + d_output;
	
	//cout << "Output: " << output << endl << endl << endl << endl;
	
	return output;
}
