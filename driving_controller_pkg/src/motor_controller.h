#ifndef _MOTOR_CONTROLLER_H_
#define _MOTOR_CONTROLLER_H_

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include<pluginlib/class_list_macros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include "ics.h"
#include "piezo_sonic.h"
#include "trapezoidal_control.h"
#include "wheel_odometry.h"

namespace driving_controller_ns
{

class MotorController
{
  private:
  int steer_now = 0;
  int steer_last = 0;
  int steer_next = 0;
  int steering_value[5][4] = {
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
  double steering_angle_now[4];    // [rad]
  
  ICSData ics_data;
  PiezoSonic piezo[4];
  
  // for steering
  double piezo_goal[4];
  bool first_steering = true;
  double Kp[4] = {8600, 2000, 7200, 9600};//{5400, 1500, 4500, 6000};  // 1800, 1000, 1500, 1800 -> 1800,1000,1500,6000
  TrapezoidalPosControl trape[4];
  bool steering_flag = false;
  double pos_p_m[4];
  double pos_s_o[4];
  
  // for running
  static constexpr double  alpha = 0.9;
  static constexpr double  beta = 0.97;
  static constexpr double  mps_to_digit = 2500.0;
  double output[4];
  
  // for odometry
  WheelOdometry odom;
  
  // Subscribe joint state
  ros::NodeHandle nh;
  ros::Subscriber joint_states_sub;
  ros::Publisher pub[8];

  sensor_msgs::JointState wheel_state;
  void joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg);
  double diff_wheel_state[4];
  sensor_msgs::JointState wheel_state_last;
  bool first_joint_states_callback = true;
  
  void drive_servo(int servo_id, int angle, int speed);
  void drive_piezo(int piezo_id, int speed);
  
  double servo_to_rad(int servo_id);

  int sign(int val) { return (val>0)-(val<0); }
  
  public:
  MotorController();
  ~MotorController();
    
  void idling();
  bool steering(int steer_next);
  void running(double speed_ms);
  bool go_to_home();

  bool check_servo_stop(int servo_id);
  bool check_all_servos_stop();
  bool check_all_piezos_stop();
  void set_piezo_goal_position(int piezo_id, int amount);
  bool check_piezo_stop(int piezo_id);
  void steering_stop();
};

}
#endif
