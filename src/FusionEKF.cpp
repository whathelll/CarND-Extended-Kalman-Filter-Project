#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <cmath>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;



/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

	//state covariance matrix P
	ekf_.P_ = MatrixXd(4, 4);
	ekf_.P_ << 1, 0, 0, 0,
			  0, 1, 0, 0,
			  0, 0, 1000, 0,
			  0, 0, 0, 1000;
	H_laser_ << 1, 0, 0, 0,
			   0, 1, 0, 0;
}


/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

//	if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
//		return;
//	}
	/*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
//    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 0, 0, 0, 0;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
    	//rho phi, pho_dot
    	previous_timestamp_ = measurement_pack.timestamp_;
    	auto rho = measurement_pack.raw_measurements_[0];
    	auto phi = measurement_pack.raw_measurements_[1];
    	ekf_.x_[0] = rho * cos(phi);
    	ekf_.x_[1] = rho * sin(phi);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
    	ekf_.x_[0] = measurement_pack.raw_measurements_[0];
    	ekf_.x_[1] = measurement_pack.raw_measurements_[1];
    	previous_timestamp_ = measurement_pack.timestamp_;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }


  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.SetF(dt);
  ekf_.SetQ(dt, 9, 9);

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
		//update Hj then call update
		//update R
		//recover state parameters
		float px = ekf_.x_(0);
		float py = ekf_.x_(1);
		float vx = ekf_.x_(2);
		float vy = ekf_.x_(3);

		//TODO: YOUR CODE HERE

		//pre-compute a set of terms to avoid repeated calculation
		float rho2 = px*px+py*py;
		float rho = sqrt(rho2);
		float rho3 = (rho2*rho);

		//check division by zero
		if(fabs(rho2) < 0.0001){
			cout << "CalculateJacobian () - Error - Division by Zero" << endl;
			Hj_ = MatrixXd(3, 4);
		} else {
			//compute the Jacobian matrix
			Hj_ << (px/rho), (py/rho), 0, 0,
				  -(py/rho2), (px/rho2), 0, 0,
				  py*(vx*py - vy*px)/rho3, px*(px*vy - py*vx)/rho3, px/rho, py/rho;
		}
		ekf_.R_ = R_radar_;
		ekf_.H_ = Hj_;
		ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Laser updates
		//update H, R then call update
		ekf_.R_ = R_laser_;
		ekf_.H_ = H_laser_;
		ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
