#ifndef _MOTOR_CONTROLLER_H_
#define _MOTOR_CONTROLLER_H_

#include <ros/ros.h>
#include "ics.h"
#include "piezo_sonic.h"

class MotorController
{
  private:
  int steer_now = 0;
  int steering_angle[][4] = {
    {0, 0, 0, 0},          // 0: Stop, F, B
    {45, 45, 45, 45},      // 1: FL, BR
    {-45, -45, -45, -45},  // 2: FR, BL
    {-90, 90, -90, 90},    // 3: L, R
    {-45, 45, -45, 45}     // 4: Rotation
  };
  int servo_angle_now[4];
  
  ICSData ics_data;
  PiezoSonic piezo[4];
  
  // for running
  constexpr double  alpha = 0.9;
  constexpr double  beta = 0.95;
  constexpr double  mps_to_digit = 1000.0;
  double speed_d = 0;
  
  // Subscribe joint state
  ros::NodeHandle nh;
  ros::Subscriber joint_states_sub;
  void joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg);
  
  void steer_servo(int motor_id, int angle, int speed);
  void steer_piezo(int motor_id, int speed);
  int angle_to_pulse(int angle);
  
  public:
  SteeringController();
  ~SteeringController();  
  void steering(int steer_next);
  void running(double speed_ms);
};

#endif
