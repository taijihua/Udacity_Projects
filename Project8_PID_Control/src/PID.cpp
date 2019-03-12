#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_, double target_val_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
	Kp = Kp_;
	Ki = Ki_;
	Kd = Kd_;

	p_error = 0;
	i_error = 0;
	d_error = 0;

	last_p_error = 0;
	N = 0;
	total_error = 0;
	target_val = target_val_;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
	p_error = cte-target_val;
	if(N>=1)
		d_error = p_error-last_p_error;
	i_error += p_error;
	last_p_error = p_error;
	//cteHistory.push_back(p_error);
	N++;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
	int cutoffN = 0;// in case i want to calculate error from n=cutoffN
	if(N>cutoffN)
		total_error +=	p_error*p_error;
  return total_error/(N-cutoffN);  // TODO: Add your total error calc here!
}
double PID::CalcOutput(){
	// calculate steering output based on cteHistory and PID coefficients
	double output = -p_error*Kp-i_error*Ki-d_error*Kd;
	return output;
}
