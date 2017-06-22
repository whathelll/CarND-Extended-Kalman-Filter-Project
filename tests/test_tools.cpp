#include "gtest/gtest.h"
#include "../src/tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;


TEST(Tools, CalculateRMSE) {
	vector<VectorXd> estimations;
	vector<VectorXd> ground_truth;

	//the input list of estimations
	VectorXd e(4);
	e << 1, 1, 0.2, 0.1;
	estimations.push_back(e);
	e << 2, 2, 0.3, 0.2;
	estimations.push_back(e);
	e << 3, 3, 0.4, 0.3;
	estimations.push_back(e);

	//the corresponding list of ground truth values
	VectorXd g(4);
	g << 1.1, 1.1, 0.3, 0.2;
	ground_truth.push_back(g);
	g << 2.1, 2.1, 0.4, 0.3;
	ground_truth.push_back(g);
	g << 3.1, 3.1, 0.5, 0.4;
	ground_truth.push_back(g);

	Tools tool;

	VectorXd result = tool.CalculateRMSE(estimations, ground_truth);
	VectorXd expected(4);
	expected << 0.1, 0.1, 0.1, 0.1;


	EXPECT_TRUE(expected.isApprox(result));
}


TEST(Tools, CalculateJacobian) {
	VectorXd x_predicted(4);
	x_predicted << 1, 2, 0.2, 0.4;

	Tools tool;
	MatrixXd Hj = tool.CalculateJacobian(x_predicted);

	MatrixXd compare(3,4);
	compare << 0.447214, 0.894427, 0, 0,
		    	-0.4, 0.2, 0, 0,
		        0, 0, 0.447214, 0.894427;

	EXPECT_TRUE((compare-Hj).norm() <= 0.000001);


}
