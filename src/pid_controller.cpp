#include "pid_controller/pid_controller.h"

using namespace pid;

PIDController::PIDController(double Kp, double Ki, double Kd, double min_limit,
                             double max_limit, double integral_limit,
                             double sample_time) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  this->min_limit = min_limit;
  this->max_limit = max_limit;
  this->last_update_time = std::chrono::system_clock::now();
  this->previous_error = 0.0;
  this->integral = 0;
  this->integral_limit = integral_limit;
  this->sample_time = sample_time;
  this->prev_process_variable = 0.0;
}

void PIDController::update_Kp(double Kp) {
  this->Kp = Kp;
  this->integral = 0.0;
}

void PIDController::update_Ki(double Ki) {
  this->Ki = Ki;
  this->integral = 0.0;
}

void PIDController::update_Kd(double Kd) {
  this->Kd = Kd;
  this->integral = 0.0;
}

void PIDController::update_integral_limit(double integral_limit) {
  this->integral_limit = integral_limit;
}

void PIDController::set_setpoint(double setpoint) { this->setpoint = setpoint; }

double PIDController::get_setpoint() { return this->setpoint; }

double PIDController::set_sample_time(double sample_time) {
  this->sample_time = sample_time;
}

void PIDController::update_pv_limits(double min_limit, double max_limit) {
  this->min_limit = min_limit;
  this->max_limit = max_limit;
}

// TODO: Update PID only if delta time is greater than some sample time
double PIDController::compute(double feedback) {
  auto current_time = std::chrono::system_clock::now();
  std::chrono::duration<double> time_elapsed = current_time - last_update_time;
  double dt = time_elapsed.count();

  if (dt >= this->sample_time) {
    double error = setpoint - feedback;

    double p_term = Kp * error;

    integral += error * dt;
    double i_term =
        Ki * (integral < integral_limit ? integral : integral_limit);
    i_term = Ki * (integral > -integral_limit ? integral : -integral_limit);

    double derivative = (error - previous_error) / dt;
    double d_term = Kd * derivative;

    double pv = p_term + i_term + d_term;

    pv = pv > max_limit ? max_limit : pv;
    pv = pv < min_limit ? 0 : pv;

    previous_error = error;
    last_update_time = current_time;
    prev_process_variable = process_variable;

    std::cout << "P: " << p_term << "   I: " << i_term << "   D: " << d_term
              << '\n';
    std::cout << "Setpoint: " << setpoint << "   PV: " << pv
              << "   Error: " << error << '\n';
    // std::cout << "Kp: " << Kp << "   Ki: " << Ki << "   Kd: " << Kd << '\n';

    return pv;
  }
  return prev_process_variable;
}
