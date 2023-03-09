#include <ros/ros.h>

class SteeringController
{
  private:
  int steer_now = 0;
  int steering_angle[][4] = {
    {0, 0, 0, 0},
    {45, 45, 45, 45},
    {-45, -45, -45, -45},
    {-90, 90, -90, 90},
    {-45, 45, -45, 45}
  };
  
  ICSData ics_data;
  
  void drive_servo(int motor_id, int angle, int speed)
  {
    ics_set_speed(&ics_data, motor_id, speed);
    int pulse = (int)(angle * 4000.0 / 135.0) + 7500;
    ics_pos(&ics_data, motor_id, pulse);
  }
  
  void drive_piezo(int motor_id, int speed)
  {
    
  }
  
  public:
  SteeringController() {
    ics_init(&ics_data);
  }
  
  void steering(int steer_next)
  {
    int amount[4];
    for(int i=0;i<4;i++) {
      amount[i] = steering_angle[steer_next][i] - steering_angle[steer_now][i];
    }
    drive_servo(0, steering_angle[steer_next][i], amount[i]/45);
    piezo();
    steer_now = steer_next;
  }
};
