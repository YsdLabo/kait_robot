#ifndef _MOTOR_CONTROLLER_H_
#define _MOTOR_CONTROLLER_H_

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include<pluginlib/class_list_macros.h>
#include <sensor_msgs/JointState.h>
#include "ics.h"
#include "piezo_sonic.h"

namespace driving_controller_ns
{

class MotorController
{
  private:
  int steer_now = 0;
  int steer_last = 0;
  int steer_next = 0;
  int steering_angle[5][4] = {
    {7500, 7500, 7500, 7500},    // 0: Stop, F, B (0,0,0,0)
    {8833, 8833, 8833, 8833},    // 1: FL, BR (45,45,45,45)
    {6167, 6167, 6167, 6167},    // 2: FR, BL (-45,-45,-45,-45)
    {4833, 10167, 4833, 10167},  // 3: L, R (-90,90,-90,90)
    {6167, 8833, 6167, 8833}     // 4: Rotation (-45,45,-45,45)
  };
  int steering_speed[5][5][4] = { //[now][next][id]
    {{ 0, 0, 0, 0},{-1, 1, 1,-1},{ 1,-1,-1, 1},{ 2, 2,-2,-2},{ 1, 1,-1,-1}},
    {{ 1,-1,-1, 1},{ 0, 0, 0, 0},{ 2,-2,-2, 2},{ 3, 1,-3,-1},{ 2, 0,-2, 0}},
    {{-1, 1, 1,-1},{-2, 2, 2,-2},{ 0, 0, 0, 0},{ 1, 3,-1,-3},{ 0, 2, 0,-2}},
    {{-2,-2, 2, 2},{-3,-1, 3, 1},{-1,-3, 1, 3},{ 0, 0, 0, 0},{-1,-1, 1, 1}},
    {{-1,-1, 1, 1},{-2, 0, 2, 0},{ 0,-2, 0, 2},{ 1, 1,-1,-1},{ 0, 0, 0, 0}}
  };
  int servo_angle_now[4];
  
  ICSData ics_data;
  PiezoSonic piezo[4];
  
  // for running
  static constexpr double  alpha = 0.9;
  static constexpr double  beta = 0.95;
  static constexpr double  mps_to_digit = 4000.0;
  double output[4];
  
  // Subscribe joint state
  ros::NodeHandle nh;
  ros::Subscriber joint_states_sub;

  sensor_msgs::JointState joint_state;
  void joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg);
  
  void drive_servo(int motor_id, int angle, int speed);
  void drive_piezo(int motor_id, int speed);
  
  int sign(int val) { return (val>0)-(val<0); }
  
  public:
  MotorController();
  ~MotorController();
    
  void steering(int steer_next);
  void running(double speed_ms);
  bool go_to_home();

  bool check_servo_stop(int id);
  bool check_all_servos_stop();
  bool check_all_piezos_stop();
};

}
#endif
