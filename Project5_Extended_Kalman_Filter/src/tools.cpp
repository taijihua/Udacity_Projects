#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
	VectorXd rmse(4);
	rmse << 0,0,0,0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if(estimations.size() != ground_truth.size()
			|| estimations.size() == 0){
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

	//accumulate squared residuals
	for(unsigned int i=0; i < estimations.size(); ++i){

		VectorXd residual = estimations[i] - ground_truth[i];

		//coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	//calculate the mean
	rmse = rmse/estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	//std::cout<<rmse<<std::endl;
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
	MatrixXd Hj = MatrixXd::Zero(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//TODO: YOUR CODE HERE

	//check division by zero
	if(px==0 && py==0)
		{
			cout << "error, check to make sure px and py are not both 0's"<< endl;
			return Hj;
		}
	//compute the Jacobian matrix
	Hj(0, 0) = px/sqrt(px*px+py*py);
	Hj(0, 1) = py/sqrt(px*px+py*py);
	Hj(1, 0) = -py/(px*px+py*py);
	Hj(1, 1) = px/(px*px+py*py);
	Hj(2, 0) = py*(vx*py-vy*px)/pow((px*px+py*py), 1.5);
	Hj(2, 1) = px*(vy*px-vx*py)/pow((px*px+py*py), 1.5);
	Hj(2, 2) = px/sqrt(px*px+py*py);
	Hj(2, 3) = py/sqrt(px*px+py*py);
	return Hj;

}

VectorXd Tools::ConvertStateToRadarMeasurement(const VectorXd& x_state)
{
	float ro;
	float theta;
	float ro_dot;
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);
	ro = sqrt(pow(px, 2)+pow(py, 2));
	if(px==0)
		theta = 0;
	else
		theta = atan2(py, px);
	ro_dot = (px*vx+py*vy)/sqrt(px*px+py*py);
	VectorXd result(3);
	result << ro, theta, ro_dot;
	return result;

}
