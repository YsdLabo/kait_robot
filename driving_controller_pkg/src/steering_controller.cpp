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
  
  void drive_servo(int motor_id, int angle, int speed)
  {
    ics_set_speed(&ics_data, motor_id, speed);
    int pulse = angle_to_pulse(angle);
    ics_pos(&ics_data, motor_id, pulse);
  }
  
  void drive_piezo(int motor_id, int speed)
  {
    // 0:-, 1:+, 2:+, 3:-
    if(motor_id == 0 || motor_id == 3) {
      
    }
    else {
    }
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
      amount[i] = (steering_angle[steer_next][i] - steering_angle[steer_now][i]) ;
    }
    drive_servo(i, steering_angle[steer_next][i], amount[i]);
    drive_piezo(i, amount[i]);
    steer_now = steer_next;
  }
  
  void running(double speed)
  {
    // forward  0:+ 1:- 2:- 3:+
  }
};
