#include <uWS/uWS.h>
#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  double abs_err;

  /*
  * Coefficients
  */ 
  double Kp_;
  double Ki_;
  double Kd_;

  double dp_p;
  double dp_i;
  double dp_d;

  int steps;
  int getSteps();

  int twiddle_interval;
  int twiddle_step;
  bool last_twiddle_up;
  bool first_twiddle;
  float best_err;

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
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Calculate the steering angle for a step
  */
  double calculateAngle();

  /*
  * Twiddle method
  */
  void TwiddleParams();

  void Restart(uWS::WebSocket<uWS::SERVER> ws);
};

#endif /* PID_H */
