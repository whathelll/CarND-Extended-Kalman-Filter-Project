
#include "../src/kalman_filter.h"
#include "gtest/gtest.h"
#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

VectorXd getX(float x, float y, float vx, float vy) {
	VectorXd result(4);
    result << x, y, vx, vy;
    return result;
}

MatrixXd getQ(float dt) {
	auto noise_ax = 9;
	auto noise_ay = 9;

	double half_dt_sq = dt*dt/2;
    MatrixXd G(4,2);

    G << half_dt_sq, 0,
         0, half_dt_sq,
         dt, 0,
         0, dt;
    MatrixXd Gtrans = G.transpose();

    MatrixXd Qv(2, 2);
    Qv << noise_ax, 0,
          0, noise_ay;

    MatrixXd Q = G*Qv*Gtrans;
    return Q;
}


TEST(KalmanFilter, SetF)
{
	KalmanFilter kf;
	kf.x_ = getX(1, 1, 2, 4);
	kf.SetF(2);
	EXPECT_EQ(kf.F_(0, 2), 2);
	EXPECT_EQ(kf.F_(1, 3), 2);
}

TEST(KalmanFilter, SetQ)
{
	KalmanFilter kf;
	kf.x_ = getX(1, 1, 2, 4);
	kf.SetQ(1, 9, 9);
	MatrixXd Q = getQ(1);
//	std::cout << kf.Q_ << std::endl;
//	std::cout << (kf.Q_-Q).norm() << std::endl;
	EXPECT_EQ(kf.Q_(0, 2), Q(0, 2));
	EXPECT_EQ(kf.Q_(1, 3), Q(1, 3));
	EXPECT_EQ((kf.Q_-Q).norm(), 0); //test all values are equal

	kf.SetQ(0.5, 9, 9);
	Q = getQ(0.5);
	EXPECT_EQ((kf.Q_-Q).norm(), 0); //test all values are equal
}

TEST(KalmanFilter, Predict)
{
	KalmanFilter kf;
	kf.x_ = getX(1, 1, 2, 4);
	kf.SetF(1);
	kf.SetQ(1, 9, 9);
	kf.P_ = MatrixXd(4, 4);
	kf.P_ << 1, 0, 0, 0,
			  0, 1, 0, 0,
			  0, 0, 1000, 0,
			  0, 0, 0, 1000;

	kf.Predict();
//	std::cout << kf.P_ << std::endl;
	//test x after 1 second
    EXPECT_EQ(kf.x_[0], 3);
    EXPECT_EQ(kf.x_[1], 5);
    EXPECT_EQ(kf.x_[2], 2);
    EXPECT_EQ(kf.x_[3], 4);
    // test p
    EXPECT_TRUE(kf.P_(0, 0) > 1000);

	kf.SetF(0.5);
	kf.Predict();
	//test x after 0.5 second
    EXPECT_EQ(kf.x_[0], 4);
    EXPECT_EQ(kf.x_[1], 7);
    EXPECT_EQ(kf.x_[2], 2);
    EXPECT_EQ(kf.x_[3], 4);
    //test P
}

TEST(KalmanFilter, Update)
{
	KalmanFilter kf;
	kf.x_ = getX(1, 1, 2, 4);
	kf.SetF(1);
	kf.SetQ(1, 9, 9);
	kf.P_ = MatrixXd(4, 4);
	kf.P_ << 1, 0, 0, 0,
			  0, 1, 0, 0,
			  0, 0, 1000, 0,
			  0, 0, 0, 1000;

	//first prediction
	kf.Predict();

	//setup for measurement
	kf.H_ = MatrixXd(2, 4);
	kf.H_ << 1, 0, 0, 0,
			   0, 1, 0, 0;
	kf.R_ = MatrixXd(2, 2);
	kf.R_ << 0.0225, 0,
	        0, 0.0225;

	VectorXd z(2);
	z << 4, 6;
	kf.Update(z);
//	std::cout << "first measurement" << std::endl;
//	std::cout << kf.x_ << std::endl;
//	std::cout << kf.P_ << std::endl;

	//next prediction assuming it's still the same 1 second
	//interval so we don't change F or Q
//	std::cout << "second predict" << std::endl;
	kf.Predict();
//	std::cout << kf.x_ << std::endl;
//	std::cout << kf.P_ << std::endl;

//	std::cout << "second measurement" << std::endl;
	z << 7, 13;
	kf.Update(z);
//	std::cout << kf.x_ << std::endl;
//	std::cout << kf.P_ << std::endl;
}



