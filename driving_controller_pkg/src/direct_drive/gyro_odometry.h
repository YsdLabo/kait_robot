#ifndef _GYRO_ODOMETRY_H_
#define _GYRO_ODOMETRY_H_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tr/tr.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>


class GyroOdometry
{
private:
	// node handle
	ros::NodeHandle  nh;
	// subscribe
	ros::Subscriber  sub_odom;
	ros::Subscriber  sub_imu;
	// publich
	ros::Publisher  pub_odom;
	// tf
	tf::TransformBroadcaster tf_broadcaster;
	// odom
	nav_msgs::Odometry odom2d_now;
	nav_msgs::Odometry odom2d_last;
	nav_msgs::Odometry odom3d_now;
	nav_msgs::Odometry odom3d_last;
	// imu
	sensor_msgs::Imu imu_last;
	// flag
	bool first_callback_odom = true;
	bool first_callback_imu = true;
public:
	GyroOdometry();
	void initialize_odom(nav_msgs::Odometry& odom);
	void callback_odom(const nav_msgs::OdometryConstPtr& msg);
	void callback_imu(const sensor_msgs::ImuConstPtr& msg);
	void Publication();
};

#endif
