#include <numeric>
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
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;

    prev_cte = 0;
    p_error = 0;
    i_error = 0;
    d_error = 0;
}

void PID::UpdateError(double cte_) {
    p_error = cte_;
    i_error += cte_;
    d_error = cte_ - prev_cte;
    prev_cte = cte_;
}

double PID::TotalError() {
    double steer = -Kp * p_error - Kd * d_error - Ki * i_error;
    return steer;
}