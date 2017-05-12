#include "PID.h"
#include <iostream>
#include <math.h>

using namespace std;


PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Kd, double Ki) {
  std::cout << "INIT " << Kp << "\t" << Kd << "\t" << Ki <<std::endl;
  Kp_ = Kp;
  Kd_ = Kd;
  Ki_ = Ki;
  steps = 0;
  twiddle_interval = 1000;
  twiddle_step = -1;
  last_twiddle_up = false;
  first_twiddle = true;
  dp_p = 0.01;
  dp_d = 0.005;
  dp_i = 0.0005;
  best_err = 1.0e6;
  abs_err = 0;
}

void PID::UpdateError(double cte) {
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
  abs_err += cte * cte;
  steps++;
}

int PID::getSteps() {
  return steps;
}

double PID::TotalError() {
  return abs_err / steps;
}

double PID::calculateAngle() {
  double angle = - Kp_ * p_error - Ki_ * i_error - Kd_ * d_error;
  if (angle < -1) {
    angle = -1;
  }
  if (angle > 1) {
    angle = 1;
  }
  return angle;
}

void PID::TwiddleParams() {
  double current_err = TotalError();
  // reset cumulative errors and steps
  abs_err = 0;
  i_error = 0;
  steps = 0;
  
  if (first_twiddle) {
     best_err = current_err;
  }
  
  // If last twiddle was successful, reset best_err and increase dp_*
  if (current_err < best_err) {
    best_err = current_err;
    if (twiddle_step % 3 == 0) {
      dp_p *= 1.1;
      std::cout << "increased dp_p to " << dp_p << std::endl;
    }
    if (twiddle_step % 3 == 1) {
      dp_d *= 1.1;
      std::cout << "increased dp_d to " << dp_d << std::endl;
    }
    if (twiddle_step % 3 == 2) {
      dp_i *= 1.1;
      std::cout << "increased dp_i to " << dp_i << std::endl;
    }
    twiddle_step += 1;
    last_twiddle_up = false;
	std::cout << "Best Kp: " << Kp_ << " Kd: " << Kd_ << " Ki: " << Ki_ << " err: " << best_err << std::endl;
	std::cout << "Deltas: " << dp_p << " " << dp_d << " " << dp_i << std::endl; 
  }
  else if (last_twiddle_up == false) {
    // decrease adjustment amounts
    if (twiddle_step % 3 == 0) {
      dp_p *= 0.9;
      Kp_ += dp_p;
      std::cout << "decreased dp_p to " << dp_p << std::endl;
    }
    if (twiddle_step % 3 == 1) {
      dp_d *= 0.9;
      Kd_ += dp_d;
      std::cout << "decreased dp_d to " << dp_d << std::endl;
    }
    if (twiddle_step % 3 == 2) {
      dp_i *= 0.9;
      Ki_ += dp_i;
      std::cout << "decreased dp_i to " << dp_i << std::endl;
    }
    twiddle_step += 1;  
    std::cout << "Reset to Kp: " << Kp_ << " Kd: " << Kd_ << " Ki: " << Ki_ << " best err: " << best_err << " curr err" << current_err << std::endl;
  }

  // twiddle param up  
  if (last_twiddle_up == false) {
    std::cout << "twiddle up param " << twiddle_step % 3 << std::endl;
    if (twiddle_step % 3 == 0) {
      Kp_ += dp_p;
    }
    if (twiddle_step % 3 == 1) {
      Kd_ += dp_d;
    }
    if (twiddle_step % 3 == 2) {
      Ki_ += dp_i;
    }
    last_twiddle_up = true;
  }
  // twidle param down
  else {
    std::cout << "twiddle down param " << twiddle_step % 3 << std::endl;
    if (twiddle_step % 3 == 0) {
      Kp_ -= 2 * dp_p;
    }
    if (twiddle_step % 3 == 1) {
      Kd_ -= 2 * dp_d;
    }
    if (twiddle_step % 3 == 2) {
      Ki_ -= 2 * dp_i;
    }
    last_twiddle_up = false;
  }
  first_twiddle = false;
}

void PID::Restart(uWS::WebSocket<uWS::SERVER> ws){
  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}

