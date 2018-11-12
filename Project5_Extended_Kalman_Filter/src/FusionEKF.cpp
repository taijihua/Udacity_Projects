#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

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
  H_laser_ << 1, 0, 0, 0,
		  0, 1, 0, 0;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


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
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
    	float ro;
    	float theta;
    	float ro_dot;
    	ro = measurement_pack.raw_measurements_(0);
    	theta = measurement_pack.raw_measurements_(1);
    	ro_dot = measurement_pack.raw_measurements_(2);
    	ekf_.x_(0) = ro*sin(theta);
    	ekf_.x_(1) = ro*cos(theta);
    	ekf_.x_(2) = ro_dot*sin(theta); //note this is not accurate, as we only know ro_dot(actual speed projection on ro direction), not the actual speed
    	ekf_.x_(3) = ro_dot*cos(theta); //same note as above

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
    	ekf_.x_(0) = measurement_pack.raw_measurements_(0);
    	ekf_.x_(1) = measurement_pack.raw_measurements_(1);
    }
    // initial (guess) of state covariance matrix
    ekf_.P_ = MatrixXd::Identity(4, 4)*10; //?? no idea what this should be, hopefully the initial value doesn't matter much as it gets evolved to close to real world...
    // the following is the value from a quiz in Udacity class, which is not better than the above
    /* ekf_.P_ << 1, 0, 0, 0,
    			  0, 1, 0, 0,
    			  0, 0, 1000, 0,
    			  0, 0, 0, 1000; */

    previous_timestamp_ = measurement_pack.timestamp_;

    ekf_.F_ = Eigen::MatrixXd(4, 4);
    ekf_.Q_ = Eigen::MatrixXd(4, 4);

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

  // need updated F and Q values for prediction step
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  //std::cout<<"dt (sec) "<<dt<<std::endl;
  previous_timestamp_ = measurement_pack.timestamp_;
  ekf_.F_ << 1, 0, dt, 0,
		  	  0, 1, 0, dt,
			  0, 0, 1, 0,
			  0, 0, 0, 1;
  //std::cout<<"ekf_.F_ "<<ekf_.F_<<std::endl;
  float noise_ax = 9;
  float noise_ay = 9;
  ekf_.Q_ << (dt*dt*dt*dt*noise_ax/4), 0, (dt*dt*dt*noise_ax/2), 0,
  	         0, (dt*dt*dt*dt*noise_ay/4), 0, (dt*dt*dt*noise_ay/2),
  	         (dt*dt*dt*noise_ax/2), 0, (dt*dt*noise_ax), 0,
  	         0, (dt*dt*dt*noise_ay/2), 0, (dt*dt*noise_ay);

  //std::cout<<"ekf_.Q_ "<<ekf_.Q_<<std::endl;
  ekf_.Predict();
  //std::cout<<"ekf_.x_ "<<ekf_.x_<<std::endl;
  //std::cout<<"ekf_.p_ "<<ekf_.P_<<std::endl;
  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */
  // need updated H and R values for measurement update
  Eigen::MatrixXd z = measurement_pack.raw_measurements_;
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
	  ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
	  ekf_.R_ = R_radar_;
	  //std::cout<<"converted radar measurement from state: "<<tools.ConvertStateToRadarMeasurement(ekf_.x_)<<std::endl;
	  ekf_.UpdateEKF(z, tools.ConvertStateToRadarMeasurement(ekf_.x_));
  } else {
    // Laser updates
	  ekf_.H_ = H_laser_;
	  ekf_.R_ = R_laser_;
	  ekf_.Update(z);
  }
  //std::cout<<"done measurement update"<<std::endl;
  //std::cout<<"ekf_.x_ "<<ekf_.x_<<std::endl;
  //std::cout<<"ekf_.p_ "<<ekf_.P_<<std::endl;

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
