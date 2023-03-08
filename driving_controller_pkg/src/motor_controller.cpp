#include<ros/ros.h>
#include<nodelet/nodelet.h>
#include<pluginlib/class_list_macros.h>
#include<driving_controller_pkg/DrivingState.h>
#include<std_srvs/Empty.h>

namespace driving_controller_ns
{
  class MotorController : public nodelet::Nodelet
  {
    private:
    ros::NodeHandle nh;
    ros::ServiceServer serverDrivingState;
    ros::ServiceServer serverStoppedState;
    ros::Timer  motor_loop;
    
    int driving_state;
    
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
  
  bool MotorController::driving_state_service(driving_controller_pkg::DrivingState::Request& req, driving_controller_pkg::DrivingState::Response& res)
  {
    driving_state = req.request.state;
  }
  
  bool MotorController::stopped_state_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    if() return true;
    return false;
  }
  
  void MotorController::motor_loop_callback(const ros::TimerEvent& e)
  {
  }
}

PLUGINLIB_EXPORT_CLASS(driving_controller_ns::MotorController, nodelet::Nodelet)
