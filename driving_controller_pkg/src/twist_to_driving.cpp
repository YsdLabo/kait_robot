#include<ros/ros.h>
#include<nodelet/nodelet.h>
#include<pluginlib/class_list_macros.h>

namespace driving_controller
{

class TwistToDriving : public nodelet::Nodelet
{
private:
	ros::NodeHandle nh;
	ros::NodeHandle pnh;
	
public:
	void OnInit()
	{
		nh = getNodeHandle();
		pnh = getPrivateNodeHandle();
	}
};

}
