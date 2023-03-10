#include <ros/ros.h>
#include "ics.h"
#include "piezo_sonic.h"

class SteeringController
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
  ros::Subscriber joint_state_sub;
  
  void steer_servo(int motor_id, int angle, int speed)
  {
    ics_set_speed(&ics_data, motor_id, std::abs(speed) * 20);
    int pulse = angle_to_pulse(angle);
    ics_pos(&ics_data, motor_id, pulse);
  }
  
  void steer_piezo(int motor_id, int speed)
  {
    int speed_m = speed;
    // 0:-, 1:+, 2:+, 3:-
    if(motor_id == 0 || motor_id == 3) {
      speed_m *= -1;
    }
    piezo[motor_id].move(speed_m);
  }
  
  int angle_to_pulse(int angle)
  {
    int pulse = (int)(angle * 4000.0 / 135.0) + 7500;
    if(pulse < 3500) pulse = 3500;
    if(pulse > 11500) pulse = 11500;
    return pulse;
  }
  
  public:
  SteeringController() {
    ics_init(&ics_data);
    for(int i=0;i<4;i++) piezo[i].open(i);
    nh = getNodeHandle();
    joint_state_sub = nh.subscribe("odom", 10, &Steering
  }
  
  ~SteeringController() {
    ics_close(&ics_data);
    delete [] piezo;
    for(int i=0;i<4;i++) piezo[i].close();
  }
  
  void steering(int steer_next)
  {
    int amount[4];
    for(int i=0;i<4;i++) {
      amount[i] = (steering_angle[steer_next][i] - steering_angle[steer_now][i]) / 45;
      steer_servo(i, steering_angle[steer_next][i], amount[i]);
      steer_piezo(i, amount[i]);
    }
    steer_now = steer_next;
  }
  
  void running(double speed_ms)
  {
    double speed = speed_ms * mps_to_digit;
    // Low-pass
    if(speed == 0) speed_d = beta * speed_d + (1.0-beta) * speed;
    else speed_d = alpha * speed_d + (1.0-alpha) * speed;
    
    // forward  0:+ 1:- 2:- 3:+
    piezo[0].move(speed_d);
    piezo[1].move(-1*speed_d);
    piezo[2].move(-1*speed_d);
    piezo[3].move(speed_d);
  }
};
