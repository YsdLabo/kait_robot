#include<ros/ros.h>
#include"pid_controller.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "motor_controller");
	
	PidController pidc;
	pidc.run();

	return 0;
}
