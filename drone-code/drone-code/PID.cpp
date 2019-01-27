#include "PID.h"

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
	double err = error(input, target);
	double p_error = err;
	//i_error += .5 * (err + previous_error) * dt;
	i_error += err * dt;
	double d_error = (err - previous_error) / dt;

	double p_output = kp * p_error;
	double i_output = ki * i_error;
	double d_output = kd * d_error;

	previous_error = err;
	double output = p_output + i_output + d_output;
	
	return output;
}
