#include "KalmanFilter.h"

KalmanFilter::KalmanFilter() {
	
}

/*double KalmanFilter::compute(double measuredAngle, double measuredRate, double dt) {
	
	//Predict the angle
	rate = measuredRate - bias;
	angle += rate * dt; //Integrating rate to get the angle

	//Updating the error covariance
	errorMatrix[0][0] += dt * ((dt * errorMatrix[1][1]) - errorMatrix[0][1] - errorMatrix[1][0] + q_angle);
	errorMatrix[0][1] -= dt * errorMatrix[1][1];
	errorMatrix[1][0] -= dt * errorMatrix[1][1];
	errorMatrix[1][1] += dt * q_bias;

	double s = errorMatrix[0][0] + r_measure; //error estimation

	double kalmanGain[2];
	kalmanGain[0] = errorMatrix[0][0] / s;
	kalmanGain[1] = errorMatrix[1][0] / s;

	double difference = measuredAngle - angle;

	//Update predictions with error
	angle += kalmanGain[0] * difference;
	bias += kalmanGain[1] * difference;

	//Temp variables to compute the covariance error
	double errorMatrix_00 = errorMatrix[0][0];
	double errorMatrix_01 = errorMatrix[0][1];

	errorMatrix[0][0] -= kalmanGain[0] * errorMatrix_00;
	errorMatrix[0][1] -= kalmanGain[0] * errorMatrix_01;
	errorMatrix[1][0] -= kalmanGain[1] * errorMatrix_00;
	errorMatrix[1][1] -= kalmanGain[1] * errorMatrix_01;

	return angle;
}*/

double KalmanFilter::kalmanX(double accAngle, double gyroRate, double dt) {
	double y, S;
	double K_0, K_1;

	KFangleX += dt * (gyroRate - x_bias);

	XP_00 += -dt * (XP_10 + XP_01) + Q_angle * dt;
	XP_01 += -dt * XP_11;
	XP_10 += -dt * XP_11;
	XP_11 += +Q_gyro * dt;

	y = accAngle - KFangleX;
	S = XP_00 + R_angle;
	K_0 = XP_00 / S;
	K_1 = XP_10 / S;

	KFangleX += K_0 * y;
	x_bias += K_1 * y;
	XP_00 -= K_0 * XP_00;
	XP_01 -= K_0 * XP_01;
	XP_10 -= K_1 * XP_00;
	XP_11 -= K_1 * XP_01;

	return KFangleX;
}

double KalmanFilter::kalmanY(double accAngle, double gyroRate, double dt) {
	float  y, S;
	float K_0, K_1;

	KFangleY += dt * (gyroRate - y_bias);

	YP_00 += -dt * (YP_10 + YP_01) + Q_angle * dt;
	YP_01 += -dt * YP_11;
	YP_10 += -dt * YP_11;
	YP_11 += +Q_gyro * dt;

	y = accAngle - KFangleY;
	S = YP_00 + R_angle;
	K_0 = YP_00 / S;
	K_1 = YP_10 / S;

	KFangleY += K_0 * y;
	y_bias += K_1 * y;
	YP_00 -= K_0 * YP_00;
	YP_01 -= K_0 * YP_01;
	YP_10 -= K_1 * YP_00;
	YP_11 -= K_1 * YP_01;

	return KFangleY;
}