#include "PID.h"
#include <iostream>

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */

  this->Kp = Kp_;
  this->Ki = Ki_;
  this->Kd = Kd_;

  p_error = 0;
  i_error = 0;
  d_error = 0;

}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  // CTE stands for cross track error
  i_error += cte;
  d_error = cte - p_error; // p_error has not been written yet, so holds the previous steps cte
  p_error = cte; // Now p_error has been updates
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */

  // From Udacity classroom PID implmentation: steer = -tau_p * cte - tau_d * diff_cte - tau_i * int_cte
  double steer = -1 * Kp * p_error - Kd * d_error - Ki * i_error;
  //std::cout <<  steer << std::endl; // Used for debugging

  if (steer > 1){ //Ensure we do not overshoot [-1,1] range
    return 1.0;
  }
  else if (steer < -1){
    return -1.0;
  }


  return steer;  // TODO: Add your total error calc here!
}
