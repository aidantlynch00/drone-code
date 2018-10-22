#pragma once

class PID {

private:
	double kp, ki, kd;
	double previous_error;
	double i_error;

public:
	PID(double kp, double ki, double kd);
	double getError(double input, double target);
	double* compute(double input, double target, double dt);
};