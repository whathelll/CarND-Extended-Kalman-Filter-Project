#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
	VectorXd rmse(4);
	rmse << 0,0,0,0;

	if(estimations.size() != ground_truth.size() || estimations.size() == 0) {
		cout << "Invalid data" << endl;
		return rmse;
	}
	//squared residuals
	for(int i=0; i < estimations.size(); ++i){
		VectorXd diff = estimations[i] - ground_truth[i];
		diff = diff.array().square();
		rmse += diff;
	}
	//mean
	rmse = rmse / estimations.size();
	rmse = rmse.array().sqrt();

	return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
	MatrixXd Hj(3,4);

	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//pre-compute a set of terms to avoid repeated calculation
	float rho_sq = px*px+py*py;
	float rho = sqrt(rho_sq);
	float rho_cube = (rho_sq*rho);

	//check division by zero
	if(fabs(rho_sq) < 0.0001){
		cout << "CalculateJacobian () - Error - Division by Zero" << endl;
		return Hj;
	}

	//compute the Jacobian matrix
	Hj << (px/rho), (py/rho), 0, 0,
		  -(py/rho_sq), (px/rho_sq), 0, 0,
		  py*(vx*py - vy*px)/rho_cube, px*(px*vy - py*vx)/rho_cube, px/rho, py/rho;

	return Hj;
}
