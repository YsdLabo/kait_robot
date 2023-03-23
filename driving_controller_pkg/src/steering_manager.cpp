#include<ros/ros.h>
#include<nodelet/nodelet.h>
#include<pluginlib/class_list_macros.h>
#include<driving_controller_pkg/DrivingState.h>
#include<std_srvs/Empty.h>

#include"motor_controller.h"

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
    
    int driving_state;
    int steering_dir;
    double driving_speed;
    bool stop_flag = true;
    bool start_flag = true;
    
    bool driving_state_service(driving_controller_pkg::DrivingState::Request&, driving_controller_pkg::DrivingState::Response&);
    bool stopped_state_service(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
    void motor_loop_callback(const ros::TimerEvent&);
    
    public:
    void onInit()
    {
      nh = getNodeHandle();
      serverDrivingState = nh.advertiseService("DrivingState", &SteeringManager::driving_state_service, this);
      serverStoppedState = nh.advertiseService("StoppedState", &SteeringManager::stopped_state_service, this);
      motor_loop = nh.createTimer(ros::Duration(0.01), &SteeringManager::motor_loop_callback, this);
    }
  };
  
  // Service
  bool SteeringManager::driving_state_service(driving_controller_pkg::DrivingState::Request& req, driving_controller_pkg::DrivingState::Response& res)
  {
    driving_state = req.state;
    steering_dir = req.steering;
    driving_speed = req.speed;
    //NODELET_INFO("get : %d : %lf", steering_dir, driving_speed);
    return true;
  }
  
  bool SteeringManager::stopped_state_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    return stop_flag;
  }
  
  // Main Loop
  void SteeringManager::motor_loop_callback(const ros::TimerEvent& e)
  {
    if(start_flag) {
      if(motor.go_to_home()) start_flag = false;
    }
    else {
      // Steering
      if(driving_state == 1)
      {
        motor.steering(steering_dir);
      }
      // Running
      else if (driving_state == 2 || driving_state == 3)
      {
        motor.running(driving_speed);
      }
      if(driving_state != 1) motor.steering_stop();
    
      // check
      if(motor.check_all_servos_stop() && motor.check_all_piezos_stop()) stop_flag = true;
     else stop_flag = false;
    }
  }
}

PLUGINLIB_EXPORT_CLASS(driving_controller_ns::SteeringManager, nodelet::Nodelet);
