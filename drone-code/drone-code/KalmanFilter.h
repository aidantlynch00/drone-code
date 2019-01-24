#pragma once

class KalmanFilter {
private:
	double q_angle = .001;
	double q_bias = .003;
	double r_measure = .03;

	double angle = 0;
	double bias = 0;
	double rate;

	double errorMatrix[2][2];

public:
	KalmanFilter();
	double compute(double measuredAngle, double measuredRate, double dt);

};