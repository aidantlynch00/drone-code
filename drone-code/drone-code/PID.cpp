#include "PID.h"

PID::PID(double _kp, double _ki, double _kd) {
	kp = _kp;
	ki = _ki;
	kd = _kd;

	previous_error = 0;
	i_error = 0;
}

double PID::getError(double input, double target) {
	return (target - input);
}

double* PID::compute(double input, double target, double dt) {
	double error = getError(input, target);
	double p_error = error;
	i_error += (error + previous_error) * dt;
	//i_error = self.i_error
	double d_error = (error - previous_error) / dt;

	double p_output = kp * p_error;
	double i_output = ki * i_error;
	double d_output = kd * d_error;

	previous_error = error;
	double* pid = new double[3];
	pid[0] = p_output;
	pid[1] = i_output;
	pid[2] = d_output;
	return pid;
}