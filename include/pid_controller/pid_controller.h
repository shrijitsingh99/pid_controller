#include "math.h"
#include <chrono>
#include <iostream>

namespace pid {
class PIDController {
private:
  double Kp;
  double Ki;
  double Kd;
  double setpoint;
  double process_variable;
  double prev_process_variable;
  double min_limit;
  double max_limit;
  std::chrono::system_clock::time_point last_update_time;
  double integral;
  double previous_error;
  double integral_limit;
  double sample_time;

public:
  PIDController(){};
  ~PIDController(){};
  PIDController(double Kp, double Ki, double Kd, double min_limit,
                double max_limit, double integral_limit, double sample_time);
  void update_Kp(double Kp);
  void update_Ki(double Ki);
  void update_Kd(double Kd);
  void update_pv_limits(double min_limit, double max_limit);
  void update_integral_limit(double integral_limit);
  void set_setpoint(double setpoint);
  double get_setpoint();
  double set_sample_time(double sample_time);
  double compute(double feedback);
};
} // namespace pid
