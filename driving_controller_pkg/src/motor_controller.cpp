#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace driving_controller
{

class MotorController
{
private:
  ros::NodeHandle nh;
  
  int steering_angle[5][] = {
    {0, 0, 0, 0},  // course F 0[deg], B 180[deg], stop
    {45, 45, 45, 45},  // course FL 45[deg], BR -135[deg]
    {-90, 90, -90, 90},  // course L 90[deg], R -90[deg]
    {-45, -45, -45, -45},  // course BL 135[deg], FR -45[deg]
    {-45, 45, -45, 45},  // rotation L, R
  };
  
public:
  void init()
  {
  }
  
  void steer(int course)
  {
    switch(course) {
      case 0:
        for(int i=0;i<4;i++) {
          servo[i].move(steering_angle[course][i]);   // 絶対角
          ultra[i].move(steering_angle[course][i+4]); // 相対角
        }
        break;
      case 45:
        break;
    }
  }
  
  void move(int course, double speed)
  {
  }
  
  void stop()
  {
  }
  
  bool check_stopped_motors()
  {
  }
};
  
}
PLUGINLIB_EXPORT_CLASS(driving_controller::MotorController, nodelet::Nodelet);
