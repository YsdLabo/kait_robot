#include<ros/ros.h>
#include<nodelet/nodelet.h>

namespace driving_controller_ns
{
  class MotorController
  {
    private:
    ros::NodeHandle nh;
    ros::serverService serverDrivingState;
    ros::serverService serverStoppedState;
    
    public:
    void onInit()
    {
    }
  };
}
