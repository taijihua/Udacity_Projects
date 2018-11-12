#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
	x_ = F_*x_;
	P_ = F_*P_*F_.transpose()+Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
	Eigen::VectorXd y;
	y = z-H_*x_;
	Eigen::MatrixXd s;
	s = H_*P_*H_.transpose()+R_;
	Eigen::MatrixXd kalmanGain;
	kalmanGain = P_*H_.transpose()*s.inverse();
	x_ = x_ + (kalmanGain*y);
	Eigen::MatrixXd I;
	I = MatrixXd::Identity(P_.rows(), P_.cols());
	P_ = (I-kalmanGain*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z, const VectorXd &PredictedMeasurement) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
	Eigen::VectorXd y;
	//y = z-H_*x_;
	y = z-PredictedMeasurement;
	if(y(1)<-3.1415926)
		y(1) += 3.1415926*2;
	if(y(1)>3.1415926)
		y(1) -= 3.1415926*2;

	// the following are same as regular KF, with the H_ being Jacobian matrix for linearity approximation.
	Eigen::MatrixXd s;
	s = H_*P_*H_.transpose()+R_;
	Eigen::MatrixXd kalmanGain;
	kalmanGain = P_*H_.transpose()*s.inverse();
	x_ = x_ + (kalmanGain*y);
	Eigen::MatrixXd I;
	I = MatrixXd::Identity(P_.rows(), P_.cols());
	P_ = (I-kalmanGain*H_)*P_;
}
