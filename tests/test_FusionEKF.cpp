#include "gtest/gtest.h"
#include "../src/FusionEKF.h"
#include <iostream>
#include <typeinfo>
#include <cmath>

MeasurementPackage getLidarPackage(long time, float x, float y) {
	MeasurementPackage meas_package;
	meas_package.sensor_type_ = MeasurementPackage::LASER;
	meas_package.raw_measurements_ = VectorXd(2);
	meas_package.raw_measurements_ << x, y;
	meas_package.timestamp_ = time;
	return meas_package;
}

MeasurementPackage getRadarPackage(long time, float rho, float phi, float rho_dot) {
	MeasurementPackage meas_package;
	meas_package.sensor_type_ = MeasurementPackage::RADAR;
	meas_package.raw_measurements_ = VectorXd(3);
	meas_package.raw_measurements_ << rho, phi, rho_dot;
	meas_package.timestamp_ = time;
	return meas_package;
}



TEST(FusionEKF, initialize_lidar) {

	FusionEKF fEKF;


	auto package = getLidarPackage(1477010445100000, 4, 2);
	fEKF.ProcessMeasurement(package);
	EXPECT_EQ(4, fEKF.ekf_.x_[0]);
	EXPECT_EQ(2, fEKF.ekf_.x_[1]);

}

TEST(FusionEKF, Lidar_measurement) {

	FusionEKF fEKF;
	auto package = getLidarPackage(1477010443000000, 0.312243, 0.58034);
	fEKF.ProcessMeasurement(package);
	EXPECT_FLOAT_EQ(0.312243, fEKF.ekf_.x_[0]);
	EXPECT_FLOAT_EQ(0.58034, fEKF.ekf_.x_[1]);


	package = getLidarPackage(1477010443100000, 1.17385, 0.481073);
	fEKF.ProcessMeasurement(package);
//	x_ =   1.17209
//	 0.481276
//	  7.81699
//	-0.900608
//	P_ = 0.0224541         0  0.204133         0
//	        0 0.0224541         0  0.204133
//	 0.204133         0   92.7917         0
//	        0  0.204133         0   92.7917
//	std::cout << fEKF.ekf_.F_ << std::endl;
//	std::cout << fEKF.ekf_.Q_ << std::endl;

}



TEST(FusionEKF, initialize_radar) {

	FusionEKF fEKF;

	std::cout << "1 radian in degrees = " << 180/M_PI << std::endl;

	float degreeToRadian = 60 * M_PI / 180;
	std::cout << "degreeToRadian " << degreeToRadian << std::endl;

	float radianToDegree = (degreeToRadian * 180) / M_PI;
	std::cout << "radianToDegree " << radianToDegree << std::endl;

	double testTan = tan(degreeToRadian);
	std::cout << "testTan " << testTan << std::endl;

	double testATan= atan(testTan) * 180 / M_PI;
	std::cout << "testATan " << testATan << std::endl;

	float rho = 5.0;
	auto package = getRadarPackage(1477010443100000, rho, degreeToRadian, 1);
	fEKF.ProcessMeasurement(package);
	EXPECT_FLOAT_EQ(rho*cos(degreeToRadian), fEKF.ekf_.x_[0]);
	EXPECT_FLOAT_EQ(rho*sin(degreeToRadian), fEKF.ekf_.x_[1]);

//	std::cout << "testCos " << cos(degreeToRadian) << std::endl;
//	std::cout << "testSin " << sin(degreeToRadian) << std::endl;
}


TEST(FusionEKF, update_radar_simdata) {

	FusionEKF fEKF;

	float rho = 1.01489;
	float phi = 0.554329;

	//initialize
	std::cout << "EKF 1st measurement" << std::endl;
//	Ground Truth =   0.859997
//	  0.600045
//	   5.19975
//	0.00179686

	auto package = getRadarPackage(1477010443050000, rho, phi, 4.89281);
	fEKF.ProcessMeasurement(package);
	std::cout << fEKF.ekf_.x_ << std::endl;
	std::cout << fEKF.ekf_.P_ << std::endl;


	std::cout << "EKF 2nd measurement" << std::endl;
//	Ground Truth =   1.37996
//	 0.600629
//	  5.19898
//	0.0107781
	package = getRadarPackage(1477010443150000, 1.04751, 0.38924, 4.51132);
	fEKF.ProcessMeasurement(package);

	std::cout << "EKF 3rd measurement" << std::endl;
//	Ground Truth =   1.89982
//	  0.60247
//	  5.19766
//	0.0269323
	package = getRadarPackage(1477010443250000, 1.6983, 0.29828, 5.20999);
	fEKF.ProcessMeasurement(package);

}



TEST(FusionEKF, update_crossing_to_negative_y) {

	FusionEKF fEKF;

	//initialize
	std::cout << "EKF 1st measurement" << std::endl;
//	Ground Truth = -4.96095
//	 1.01182
//	-2.02019
//	-4.76468
	auto package = getRadarPackage(1477010456450000, 5.10777, 2.92274, 1.07201);
	fEKF.ProcessMeasurement(package);
	std::cout << fEKF.ekf_.x_ << std::endl;
	std::cout << fEKF.ekf_.P_ << std::endl;


	std::cout << "EKF 2nd measurement" << std::endl;
//	Ground Truth = -5.16621
//	0.536876
//	-2.08471
//	-4.73127
	package = getRadarPackage(1477010456550000, 4.85074, 3.10427, 1.75578);
	fEKF.ProcessMeasurement(package);

	std::cout << "EKF 3rd measurement" << std::endl;
//	Ground Truth =   -5.3782
//	0.0654719
//	 -2.15477
//	 -4.69374
	package = getRadarPackage(1477010456650000, 6.00513, 3.19003, 1.77637);
	fEKF.ProcessMeasurement(package);


	std::cout << "EKF 4th measurement" << std::endl;
//	Ground Truth =  -5.59748
//	-0.401971
//	 -2.23014
//	 -4.65185
	package = getRadarPackage(1477010456750000, 5.64632, -3.11599, 2.50614);
	fEKF.ProcessMeasurement(package);

}


