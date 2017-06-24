#include "kalman_filter.h"
#include <iostream>
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;// state
  P_ = P_in;// state covariance
  F_ = F_in;// state transition
  H_ = H_in;// measurement matrix
  R_ = R_in;// measurement covariance
  Q_ = Q_in;// process covariance
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
	x_ = F_ * x_;
	P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

	VectorXd y = z - H_ * x_;
	MatrixXd HTrans = H_.transpose();
	MatrixXd S = H_ * P_ * HTrans + R_;
	MatrixXd K = P_ * HTrans * S.inverse();

	x_ = x_ + K * y;

	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
	float px = x_(0);
	float py = x_(1);
	float vx = x_(2);
	float vy = x_(3);

	float rho2 = px*px+py*py;
	float rho = sqrt(rho2);
	float phi = atan2(py, px);

	float rho_dot = (px*vx + py*vy) / rho;

	VectorXd Hx(3);
	Hx << rho, phi, rho_dot;
//	std::cout << "Hx: " << Hx << std::endl;

	VectorXd y = z - Hx;
	//seems radar measurements come in both 3.19 (higher than pi) and -3.1 so we have to fix y
	if(abs(y(1)) > M_PI) {
		y(1) = remainder(y(1), 2*M_PI);
	}

//	std::cout << "---y: " << y << std::endl;

	MatrixXd HTrans = H_.transpose();
	MatrixXd S = H_ * P_ * HTrans + R_;
	MatrixXd K = P_ * HTrans * S.inverse();

	x_ = x_ + K * y;

	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

/**
 * Set the time difference for the F matrix
 * @param dt the time difference in seconds
 */
void KalmanFilter::SetF(float dt) {
	F_ = MatrixXd(4, 4);
	F_ << 1, 0, dt, 0,
		 0, 1, 0, dt,
		 0, 0, 1, 0,
		 0, 0, 0, 1;
}

/**
 * Set the process covariance matrix
 * @param dt the time difference in seconds
 * @param noise_ax acceleration noise x
 * @param noise_ay acceleration noise y
 */
void KalmanFilter::SetQ(float dt, float noise_ax, float noise_ay){
	float dt_2 = dt * dt;
	float dt_3 = dt_2 * dt;
	float dt_4 = dt_3 * dt;

	//set the process covariance matrix Q
	Q_ = MatrixXd(4, 4);
	Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
			   0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
			   dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
			   0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
}
