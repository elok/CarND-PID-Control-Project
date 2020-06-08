#include "PID.h"

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

  this->prev_cte = 0.0;
  this->int_cte = 0.0;
  this->diff_cte = 0.0;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  diff_cte = cte - prev_cte;
  prev_cte = cte;
  int_cte += cte;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  // total error is the steering angle
  return (-Kp * prev_cte) - (Ki * int_cte) - (Kd * diff_cte);
}