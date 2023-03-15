#ifndef _MOTOR_CONTROLLER_H_
#define _MOTOR_CONTROLLER_H_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "ics.h"
#include "piezo_sonic.h"

class MotorController
{
  private:
  int steer_now = 0;
  int steering_angle[][4] = {
    {7500, 7500, 7500, 7500},    // 0: Stop, F, B (0,0,0,0)
    {8833, 8833, 8833, 8833},    // 1: FL, BR (45,45,45,45)
    {6167, 6167, 6167, 6167},    // 2: FR, BL (-45,-45,-45,-45)
    {4834, 10166, 4834, 10166},  // 3: L, R (-90,90,-90,90)
    {6167, 8833, 6167, 8833}     // 4: Rotation (-45,45,-45,45)
  };
  int servo_angle_now[4];
  
  ICSData ics_data;
  PiezoSonic piezo[4];
  
  // for running
  constexpr double  alpha = 0.9;
  constexpr double  beta = 0.95;
  constexpr double  mps_to_digit = 1000.0;
  double output[4];
  
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
