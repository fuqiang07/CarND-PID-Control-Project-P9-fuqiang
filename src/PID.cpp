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
void PID::Init(double Kp, double Ki, double Kd, double Max_Output) {
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

    //rest i_output_ to zero
    //the reason that we choose to use i_output_ not the i_error_ is:
    //it is hard for us to choose a threshold for i_error_
    //but i_output_ value can be easily limited within 0.8*max_control
    //e.g. the maximum value of steer is 1, we can set maximum value for i_output_ to 0.8
    i_error_ = 0.0;
    i_output_ = 0.0;

    max_i_output_ = 0.8 * Max_Output;

    // initially set to false, set to true in first call of ProcessMeasurement
    is_initialized_ = false;

    // Twiddling parameters
    flag_twiddle = false;
    dp = {0.1*Kp,0.1*Kd,0.1*Ki};
    step = 1;
    param_index = 2;  // this will wrao back to 0 after the first twiddle loop
    n_settle_steps = 100;
    n_eval_steps = 2000;
    total_error = 0;
    best_error = std::numeric_limits<double>::max();
    tried_adding = false;
    tried_subtracting = false;
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
        d_error_ = 0.0;
        p_error_ = -cte;
        i_error_ = -cte;

        i_output_ = -cte * Ki_;

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
    i_error_ = i_error_ + (-cte);

    i_output_ += -cte * Ki_;

    if(i_output_ > max_i_output_){
        i_output_ = max_i_output_;
    }else if(i_output_ < -max_i_output_){
        i_output_ = -max_i_output_;
    }

    // update total_error only if the vehicle have past the settling process
    if (step % (settle_steps + eval_steps) > settle_steps){
        total_error += pow(cte,2);
    }

    // Twiddle loop process:
    //
    // 1. Compare current total error with best error,
    //    if total error > best error, whether current parameter will be increased or decreased depend on the state of the flag;
    //    if parameter both flags are false, increase current parameter by dp and set increase flag to true;
    //    if increase flag is true, decrease current parameter by 2*dp and set decrease flag to true;
    //    if both flags are true, change shrink dp by 10% and set both flags to false.
    // 2. if total error < best error, refresh best error, switch to next parameter and set both flag to false.
    if (twiddle && step % (settle_steps + eval_steps) == 0){
        cout << "step: " << step << endl;
        cout << "total error: " << total_error << endl;
        cout << "best error: " << best_error << endl;
        if (total_error < best_error) {
            cout << "Parameter improved, index:" << param_index << endl;
            best_error = total_error;
            if (step !=  settle_steps + eval_steps) {
                // don't do this if it's the first loop of twiddle
                dp[param_index] *= 1.1;
            }
            // switch to next parameter
            param_index = (param_index + 1) % 3;
            increased_flag = decreased_flag = false;
        }
        if (!increased_flag && !decreased_flag) {
            // increase params[i] by dp[i]
            AddToParameterAtIndex(param_index, dp[param_index]);
            increased_flag = true;
        }
        else if (increased_flag && !decreased_flag) {
            // decrease params[i] by dp[i], multiply by 2 because we already add dp[i] in previous loop
            AddToParameterAtIndex(param_index, -2 * dp[param_index]);
            decreased_flag = true;
        }
        else {
            // shrink dp[i], move on to next parameter
            AddToParameterAtIndex(param_index, dp[param_index]);
            dp[param_index] *= 0.9;
            // next parameter
            param_index = (param_index + 1) % 3;
            increased_flag = decreased_flag = false;
        }
        total_error = 0;
        cout << "new parameters" << endl;
        cout << "P: " << Kp << ", I: " << Ki << ", D: " << Kd << endl;
    }
    step++;
    if (twiddle && step % (n_settle_steps + n_eval_steps) == 0){
        cout << "step: " << step << endl;
        cout << "total error: " << total_error << endl;
        cout << "best error: " << best_error << endl;
    }
}

/*
 * Calculate the total PID error.
 */
double PID::TotalError() {
    //return Kp_ * p_error_ + i_output_ + Kd_ * d_error_;
    return Kp_ * p_error_ + Ki_ * i_error_ + Kd_ * d_error_;
}

void PID::AddToParameterAtIndex(int index, double amount) {
    if (index == 0) {
        Kp += amount;
    }
    else if (index == 1) {
        Kd += amount;
    }
    else if (index == 2) {
        Ki += amount;
    }
    else {
        std::cout << "AddToParameterAtIndex: index out of bounds";
    }
}

