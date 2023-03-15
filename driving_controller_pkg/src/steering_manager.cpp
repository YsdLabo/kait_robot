#include<ros/ros.h>
#include<nodelet/nodelet.h>
#include<pluginlib/class_list_macros.h>
#include<driving_controller_pkg/DrivingState.h>
#include<std_srvs/Empty.h>

#include"motor_controller.h"
#include"piezo_sonic.h"
#include"ics.h"

namespace driving_controller_ns
{
  class SteeringManager : public nodelet::Nodelet
  {
    private:
    ros::NodeHandle nh;
    ros::ServiceServer serverDrivingState;
    ros::ServiceServer serverStoppedState;
    ros::Timer  motor_loop;
    
    MotorController motor;
    int steering_dir;
    double driving_speed;
    
    bool driving_state_service(driving_controller_pkg::DrivingState::Request&, driving_controller_pkg::DrivingState::Response&);
    bool stopped_state_service(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
    void motor_loop_callback(const ros::TimerEvent&);
    
    public:
    void onInit()
    {
      nh = getNodeHandle();
      serverDrivingState = nh.advertiseService("DrivingState", &MotorController::driving_state_service, this);
      serverStoppedState = nh.advertiseService("StoppedState", &MotorController::stopped_state_service, this);
      motor_loop = nh.createTimer(ros::Duration(0.01), &MotorController::motor_loop_callback, this);
    }
  };
  
  bool SteeringManager::driving_state_service(driving_controller_pkg::DrivingState::Request& req, driving_controller_pkg::DrivingState::Response& res)
  {
    steering_dir = req.request.steering;
    driving_speed = req.request.speed;
  }
  
  bool SteeringManager::stopped_state_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    if() return true;
    return false;
  }
  
  void SteeringManager::motor_loop_callback(const ros::TimerEvent& e)
  {
    // Steering
    if(std::fabs(driving_speed) < 0.01)
    {
      motor.steering(steering_dir);
    }
    // Running
    else{
      motor.running(driving_speed);
    }
    // check
  }
}

PLUGINLIB_EXPORT_CLASS(driving_controller_ns::SteeringManager, nodelet::Nodelet)
