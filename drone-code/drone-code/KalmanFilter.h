#pragma once

class KalmanFilter {
private:
	double Q_angle = 0.01;
	double Q_gyro = 0.0003;
	double R_angle = 0.01;
	double x_bias = 0;
	double y_bias = 0;
	double XP_00 = 0, XP_01 = 0, XP_10 = 0, XP_11 = 0;
	double YP_00 = 0, YP_01 = 0, YP_10 = 0, YP_11 = 0;
	double KFangleX = 0.0;
	double KFangleY = 0.0;


	double errorMatrix[2][2];

public:
	KalmanFilter();
	//double compute(double measuredAngle, double measuredRate, double dt);
	double kalmanX(double accAngle, double gyroRate, double dt);
	double kalmanY(double accAngle, double gyroRate, double dt);
};