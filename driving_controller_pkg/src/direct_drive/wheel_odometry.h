#ifndef _WHEEL_ODOMETRY_
#define _WHEEL_ODOMETRY_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

namespace driving_controller_ns
{

class WheelOdometry
{
private:
	ros::NodeHandle nh;
	ros::Publisher odom_pub;

	static constexpr double L1 = 0.4 + 0.1;
	static constexpr double L2 = 0.4 * std::sqrt(2) + 0.1;
	static constexpr double L3 = 0.2 * std::sqrt(2) + 0.05;
	static constexpr double AREA_F_L = M_PI / 8.0;			// 22.5
	static constexpr double AREA_L_F = M_PI * 3.0 / 8.0;		// 67.5
	static constexpr double AREA_L_B = M_PI * 5.0 / 8.0;		// 112.5
	static constexpr double AREA_B_L = M_PI * 7.0 / 8.0;		// 157.5
	static constexpr double AREA_F_R = - M_PI / 8.0;		// -22.5
	static constexpr double AREA_R_F = - M_PI * 3.0 / 8.0;		// -67.5
	static constexpr double AREA_R_B = - M_PI * 5.0 / 8.0;		// -112.5
	static constexpr double AREA_B_R = - M_PI * 7.0 / 8.0;		// -157.5
	static constexpr double wheel_radius = 0.04975;   // D=0.0995
	
	ros::Time  current_time;
	double cur_x;
	double cur_y;
	double cur_th;
	double cur_vx;
	double cur_vy;
	double cur_w;
	nav_msgs::Odometry odom;
	tf::TransformBroadcaster tf_caster;
	
	sensor_msgs::JointState wheel_state_last;
	bool first_run = true;

	void publish_odom();
	void publish_tf();

public:
	WheelOdometry();
	void update(sensor_msgs::JointState& wheel_state);
	void update(sensor_msgs::JointState& wheel_state, double steer[4]);
};

}

#endif
