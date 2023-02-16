#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace driving_controller
{

class MotorController
{
private:
  ros::NodeHandle nh;
  ros::Subscriber sub_course;
  
  int steering_angle[5][] = {
    {0, 0, 0, 0},  // course F 0[deg], B 180[deg], stop
    {45, 45, 45, 45},  // course FL 45[deg], BR -135[deg]
    {-90, 90, -90, 90},  // course L 90[deg], R -90[deg]
    {-45, -45, -45, -45},  // course BL 135[deg], FR -45[deg]
    {-45, 45, -45, 45},  // rotation L, R
  };
  
  void callback(const std_msgs::Int32& msg)
  {
    msg->state;
    course_next = msg->course;
    msg->speed;
  }
  
  void timer_callback()
  {
    switch(state)
    {
      case 0: // 停止
        for(int i=0;i<4;i++) {
          servo[i].stop();
          ultra[i].stop();
        }
        break;
      case 1: // 操舵
        for(int i=0;i<4;i++) {
          servo[i].move(steering_angle[course][i]);   // 絶対角
          ultra[i].move(steering_angle[course][i]); // 相対角
        }
        break;
      case 2: // 走行
        for(int i=0;i<4;i++) {
          servo[i].stop();
          ultra[i].run(speed);
        }
        break;
    }
  }
  
  bool stopService(driving_controller_pkg::check_stop::Request& req, driving_controller_pkg::check_stop::Response& res)
  {
  }
  
public:
  void onInit()
  {
    nh = getNodeHandle();
    
    sub_course = nh.subscribe("~", &MotorController::callback, this);
    timer = ros::createTimer(ros::Duration(0.01), &MotorController::timer_callback, this);
  }
  
  void steer(int course_next)
  {
    // 目標値計算
    // 
    // スタート
    state = 1;
  }
  
  void move(int course_next, double speed)
  {
    
    state = 2;
  }
  
  void stop()
  {
  }
  
  // サービスで駆動
  bool check_stopped_motors()
  {
  }
};
  
}
PLUGINLIB_EXPORT_CLASS(driving_controller::MotorController, nodelet::Nodelet);
