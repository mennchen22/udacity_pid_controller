#include <numeric>
#include <values.h>
#include <iostream>
#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() = default;

PID::~PID() = default;

void PID::Init(double Kp_, double Ki_, double Kd_, double Dp_, double Dd_, double Di_, double train_thr = 0.3) {
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

    // init train parameters
    best_err = MAXINT;
    thr = train_thr;
    parameter_ind = 0;
    state_ind = 0;

    d[0] = Dp_;
    d[1] = Dd_;
    d[2] = Di_;

    p[0] = Kp;
    p[1] = Kd;
    p[2] = Ki;
}

void PID::UpdateError(double cte_) {
    p_error = cte_;
    i_error += cte_;
    d_error = cte_ - prev_cte;
    prev_cte = cte_;
}

double PID::TotalError() const {
    double steer = -Kp * p_error - Kd * d_error - Ki * i_error;
    return steer;
}

bool PID::TrainParameters(double cte) {
    double sum = 0;
    for (double i : d) {
        sum += i;
    }
    if (sum <= thr) { // && cte <= best_err * 4
        std::cout << "Dp sum: " << sum << std::endl;
        return true;
    }

    int i = parameter_ind;

    p[i] += d[i];
    if (cte < best_err) {
        best_err = cte;
        d[i] *= 1.1;
    } else {
        if (state_ind == 0) {
            p[i] -= 2 * d[i];
        } else {
            p[i] += d[i];
            d[i] *= 0.9;
        }
    }

    // Save new values
    Kp = p[0];
    Kd = p[1];
    Ki = p[2];


    // Simulate for loop over state changes (two times per parameter)
    state_ind = (state_ind + 1) % 2;
    if (state_ind == 1) {
        parameter_ind = (parameter_ind + 1) % 3;
    }

    return false;
}

void PID::PrintParameters() const {
    std::cout << "PID: Kp: " << Kp << " Kd: " << Kd << " Ki: " << Ki << std::endl;
}
