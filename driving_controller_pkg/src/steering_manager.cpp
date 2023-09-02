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
    bool start_flag = false;
    bool block = false;
    
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
    if(block == true) {NODELET_INFO("block"); return false;}
    driving_state = req.state;
    steering_dir = req.steering;
    driving_speed = req.speed;
    if(driving_state == 1 || driving_state == 2) stop_flag = false;
    return true;
  }
  
  bool SteeringManager::stopped_state_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    //NODELET_INFO("send : stop_flag %d", (int)stop_flag);
    return stop_flag;
  }
  
  // Main Loop
  void SteeringManager::motor_loop_callback(const ros::TimerEvent& e)
  {
    // 初期実行
    if(!start_flag) {
      if(motor.go_to_home()) start_flag = true;
      stop_flag = true;
    }
    else {
      block = true;
//    NODELET_INFO("[steering manager] start : %d : %d : %lf : %d", driving_state, steering_dir, driving_speed, stop_flag);
      // Steering
      if(driving_state == 1)
      {
        if(motor.steering(steering_dir)) { // && motor.check_all_piezos_stop()){// && motor.check_all_servos_stop()) {
//        printf("[steering manager] end steering\n");
          driving_state = -1;  // to idling
          stop_flag = true;
        }
        else stop_flag = false;

      }
      // Running & Stopping
      else if (driving_state == 2 || driving_state == 3)
      {
        motor.running(driving_speed);
        // check
        if(motor.check_all_servos_stop() && motor.check_all_piezos_stop()) {
          stop_flag = true;
        }
        else stop_flag = false;
      }
      // Idling
      else {
      	motor.idling();
        stop_flag = true;
      }
      //if(driving_state != 1) {
      //  motor.steering_stop();
      //}
//    NODELET_INFO("[steering manager] end   : %d : %d : %lf : %d", driving_state, steering_dir, driving_speed, stop_flag);
      block = false;
    }
  }
}

PLUGINLIB_EXPORT_CLASS(driving_controller_ns::SteeringManager, nodelet::Nodelet);
