#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "driving_controller_test");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	ros::Rate loop(10);

	double x,y,w;
	
	sleep(5.0);

	while(ros::ok()) {
		printf("Input cmd_vel: x  -- If you finish program, input 999\n");
		scanf("%lf", &x);
		if(x > 998) break;
		printf("Input cmd_vel: y\n");
		scanf("%lf", &y);
		printf("Input cmd_vel: yaw\n");
		scanf("%lf", &w);
		
		geometry_msgs::Twist cmd_vel;
		cmd_vel.linear.x = x;
		cmd_vel.linear.y = y;
		cmd_vel.angular.z = w;
		pub.publish(cmd_vel);
	
		ros::spinOnce();
		loop.sleep();
	}
	
	return 0;
}
