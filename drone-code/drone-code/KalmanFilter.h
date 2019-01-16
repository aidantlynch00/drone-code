#pragma once

class KalmanFilter {
private:
	double q_angle = .01;
	double q_gyro = .0003;
	double r_angle = .01;
	double x_bias = 0;
	double y_bias = 0;
	double xp_00 = 0, xp_01 = 0, xp_10 = 0, xp_11 = 0;
	double yp_00 = 0, yp_01 = 0, yp_10 = 0, yp_11 = 0;
	double kf_angleX = 0;
	double kf_angleY = 0;

public:

	double kalmanFilterX(double accAngle, double gyroRate, double dt) {
		double  y, S;
		double k_0, k_1;


		kf_angleX += dt * (gyroRate - x_bias);

		xp_00 += -dt * (xp_10 + xp_01) + q_angle * dt;
		xp_01 += -dt * xp_11;
		xp_10 += -dt * xp_11;
		xp_11 += q_gyro * dt;

		y = accAngle - kf_angleX;
		S = xp_00 + r_angle;
		k_0 = xp_00 / S;
		k_1 = xp_10 / S;

		kf_angleX += k_0 * y;
		x_bias += k_1 * y;
		xp_00 -= k_0 * xp_00;
		xp_01 -= k_0 * xp_01;
		xp_10 -= k_1 * xp_00;
		xp_11 -= k_1 * xp_01;

		return kf_angleX;
	}

	double kalmanFilterY(double accAngle, double gyroRate, double dt) {
		double y, s;
		double k_0, k_1;

		kf_angleY += dt * (gyroRate - y_bias);

		yp_00 += -dt * (yp_10 + yp_01) + q_angle * dt;
		yp_01 += -dt * yp_11;
		yp_10 += -dt * yp_11;
		yp_11 += q_angle * dt;

		y = accAngle - kf_angleY;
		s = yp_00 * r_angle;
		k_0 = yp_00 / s;
		k_1 = yp_10 / s;

		kf_angleY += k_0 * y;
		y_bias += k_1 * y;
		yp_00 -= k_0 * yp_00;
		yp_01 -= k_0 * yp_01;
		yp_10 -= k_1 * yp_00;
		yp_11 -= k_1 * yp_01;

		return kf_angleY;
	}
};