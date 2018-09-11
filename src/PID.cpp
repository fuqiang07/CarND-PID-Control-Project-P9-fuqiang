#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

/*
* Initialize PID.
*/
void PID::Init(double Kp, double Ki, double Kd) {
    //Set PID parameters to assigned values
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;

    /*
    //Set errors to zeros
    p_error_ = 0.0;
    i_error_ = 0.0;
    d_error_ = 0.0;
     */

    // initially set to false, set to true in first call of ProcessMeasurement
    is_initialized_ = false;
}

/*
* Update the PID error variables given cross track error.
*/
void PID::UpdateError(double cte) {

    /*****************************************************************************
    *  Initialization
     *  The reason to utilize initialization here is to avoid a first jump in d_error_,
     *  the d_error_ should be zero at the first measurement.
    ****************************************************************************/
    if (!is_initialized_) {

        // first measurement
        p_error_ = -cte;
        i_error_ = -cte;
        d_error_ = 0.0;

        // done initializing,
        is_initialized_ = true;
        return;
    }

    /*****************************************************************************
    *  Update Error
    ****************************************************************************/
    //when p_error_ is not updated, it keeps the value of the previous error
    d_error_ = -cte - p_error_;

    p_error_ = -cte;

    i_error_ += -cte;
}

/*
 * Calculate the total PID error.
 */
double PID::TotalError() {
    return Kp_ * p_error_ + Ki_ * i_error_ + Kd_ * d_error_;
}

