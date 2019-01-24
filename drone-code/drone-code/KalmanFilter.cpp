#include "KalmanFilter.h"

KalmanFilter::KalmanFilter() {
	q_angle = .001;
	q_bias = .003;
	r_measure = .03;

	angle = 0;
	bias = 0;
	rate = 0;

	errorMatrix[0][0] = 0;
	errorMatrix[0][1] = 0;
	errorMatrix[1][0] = 0;
	errorMatrix[1][1] = 0;
}

double KalmanFilter::compute(double measuredAngle, double measuredRate, double dt) {

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
}