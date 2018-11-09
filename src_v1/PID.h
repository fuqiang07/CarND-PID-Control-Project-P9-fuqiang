#ifndef PID_H
#define PID_H

class PID {
public:
    ///* initially set to false, set to true in first call of ProcessMeasurement
    bool is_initialized_;

  /*
  * Errors
  */
  double p_error_;
  //double i_error_;
  double d_error_;

  /*
  * Coefficients
  */ 
  double Kp_;
  double Ki_;
  double Kd_;

  ///* the control output generated by Integral control
  double i_output_;

  double max_i_output_;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd, double Max_Output);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
