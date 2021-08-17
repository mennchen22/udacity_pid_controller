#ifndef PID_H
#define PID_H

#include <vector>

class PID {
public:
    /**
     * Constructor
     */
    PID();

    /**
     * Destructor.
     */
    virtual ~PID();

    /**
     * Initialize PID.
     * @param (Kp_, Ki_, Kd_) The initial PID coefficients
     */
    void Init(double Kp_, double Ki_, double Kd_, double Dp_, double Dd_, double Di_,  double train_thr);

    /**
     * Update the PID error variables given cross track error.
     * @param cte The current cross track error
     */
    void UpdateError(double cte);

    /**
     * Calculate the total PID error.
     * @output The total PID error
     */
    double TotalError() const;

    bool TrainParameters(double cte);

    void PrintParameters() const;

private:
    /**
     * PID Errors
     */
    double p_error;
    double i_error;
    double d_error;
    double prev_cte;

    /**
     * Throttle train parameter
     */
    double best_err;
    double thr;
    int parameter_ind;
    int state_ind;


    /**
     * PID Coefficients
     */
    double Kp;
    double Ki;
    double Kd;

    std::vector<double> int_cte;
    double p[3];
    double d[3];

};

#endif  // PID_H