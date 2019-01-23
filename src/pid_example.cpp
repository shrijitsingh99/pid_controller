#include "pid_controller/pid_controller.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <dynamic_reconfigure/server.h>
#include <iostream>
#include <pid_controller/PIDConfig.h>
#include <unistd.h>

auto pid_c = pid::PIDController(0.01, 0.01, 0.001, -1000.0, 10.0, 500.0, 0.1);

void callback(pid_controller::PIDConfig &config, uint32_t level) {
  // ROS_INFO("Reconfigure Request: %f %f %f %f",
  //           config.Kp,
  //           config.Ki,
  //           config.Kd,
  //           config.setpoint);

  if (!config.Kp_constant)
    pid_c.update_Kp(config.Kp);
  if (!config.Ki_constant)
    pid_c.update_Ki(config.Ki);
  if (!config.Kd_constant)
    pid_c.update_Kd(config.Kd);

  pid_c.update_integral_limit(config.integral_limit);
  pid_c.set_setpoint(config.setpoint);
  pid_c.set_sample_time(config.sample_time);
  pid_c.update_pv_limits(config.min_limit, config.max_limit);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pid_controller_node");
  ros::NodeHandle n;
  ros::Publisher feedback_pub =
      n.advertise<std_msgs::Float32>("feedback", 1000);
  ros::Publisher setpoint_pub =
      n.advertise<std_msgs::Float32>("setpoint", 1000);

  dynamic_reconfigure::Server<pid_controller::PIDConfig> server;
  dynamic_reconfigure::Server<pid_controller::PIDConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);

  server.setCallback(f);

  double pv = 0;
  double feedback = 0;

  while (ros::ok()) {
    std_msgs::Float32 feedback_msg;
    std_msgs::Float32 setpoint_msg;
    std::cout << '\n';
    pv = pid_c.compute(feedback);
    feedback = pv;
    std::cout << "Feedback: " << feedback << "   PV: " << pv << '\n';
    feedback_msg.data = feedback;
    setpoint_msg.data = pid_c.get_setpoint();
    feedback_pub.publish(feedback_msg);
    setpoint_pub.publish(setpoint_msg);
    usleep(100 * 1000);
    ros::spinOnce();
  }

  return 0;
}
