#include "gyro_odometry.h"

GyroOdometry::GyroOdometry()
{
	sub_odom = nh.subscribe("/wheel_odom", 1, &GyroOdometry::callback_odom, this);
	sub_imu = nh.subscribe("/imu/data", 1, &GyroOdometry::callback_imu, this);
	pub_odom = nh.subscribe("/gyro_odom", 1);
	initialize_odom(odom3d_now);
	initialize_odom(odom3d_last);
}

void GyroOdometry::initialize_odom(nav_msgs::Odometry& odom)
{
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_link";
	odom.pose.pose.position.x = 0.0;
	odom.pose.pose.position.y = 0.0;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation.x = 0.0;
	odom.pose.pose.orientation.y = 0.0;
	odom.pose.pose.orientation.z = 0.0;
	odom.pose.pose.orientation.w = 1.0;
}

void GyroOdometry::callback_odom(const nav_msgs::OdometryConstPtr& msg)
{
	odom2d_now = *msg;
	
	if(!first_callback_odom) {
		// 2D to 3D
		tf::Quaternion q_pose2d_last;
		tf::Quaternion q_pose3d_last;
		quaternionMsgToTF(odom2d_last.pose.pose.orientation, q_pose2d_last);
		quaternionMsgToTF(odom3d_last.pose.pose.orientation, q_pose3d_last);
		
		tf::Quaternion q_global_mode2d(
			odom2d_now.pose.pose.position.x - odom2d_last.pose.pose.position.x,
			odom2d_now.pose.pose.position.y - odom2d_last.pose.pose.position.y,
			0.0
			0.0
		);
		tf::Quaternion q_local_move2d = q_pose2d_last.inverse() * q_global_move2d * q_pose2d_last;
		tf::Quaternion q_global_move3d = q_pose3d_last * q_local_move2d * q_pose3d_last.inverse();
		
		// integration
		odom3d_now.pose.pose.position.x = odom3d_last.pose.pose.position.x + q_global_move3d.x();
		odom3d_now.pose.pose.position.y = odom3d_last.pose.pose.position.y + q_global_move3d.y();
		odom3d_now.pose.pose.position.z = odom3d_last.pose.pose.position.z + q_global_move3d.z();
	}
	
	odom2d_last = odom2d_now;
	odom3d_last = odom3d_now;
	first_callback_odom = false;
	
	Publication();
}

void GyroOdometry::callback_imu(const sensor_msgs::ImuConstPtr& msg)
{
	// Get dt
	ros::Time time_imu_now = msg->header.stamp;
	ros::Time time_imu_last = imu_last.header.stamp;
	double dt;
	try {
		dt = (time_imu_now - time_imu_last).toSec();
	}
	catch (std::runtime_error& ex) {
		ROS_ERROR("Exception: [%s]", ex.what());
	}
	
	// Orientation Estimation
	if(!first_callback_imu) {
		double delta_r = (msg->angular_velocity.x + imu_last.angular_velocity.x) * dt / 2.0;
		double delta_p = (msg->angular_velocity.y + imu_last.angular_velocity.y) * dt / 2.0;
		double delta_y = (msg->angular_velocity.z + imu_last.angular_velocity.z) * dt / 2.0;
		
		tf::Quaternion q_relative_rotation = tf::createQuaternionFromRPY(delta_r, delta_p, delta_y);
		tf::Quaternion q_pose3d_now;
		quaternionMsgToTF(odom3d_now.pose.pose.orientation, q_pose3d_now);
		q_pose3d_now = q_pose3d_now * q_relative_rotation;
		q_pose3d_now.normalize();
		quaternionTFToMsg(q_pose3d_now, odom3d_now.pose.pose.orientation);
	}
	
	imu_last = *msg;
	first_callback_imu = false;
	
	Publication();
}

void GyroOdometry::Publication()
{
	// publish odom
	odom3d_now.header.stamp = odom2d_now.header.stamp;
	pub_odom.publish(odom3d_now);
	
	// tf broadcast
	geometry_msgs::TransformStamped transform;
	transform.header.stamp = odom2d_now.header.stamp;
	transform.header.frame_id = "/odom";
	transform.child_frame_id = "/base_link";
	transform.transform.translation.x = odom3d_now.pose.pose.position.x;
	transform.transform.translation.y = odom3d_now.pose.pose.position.y;
	transform.transform.translation.z = odom3d_now.pose.pose.position.z;
	transform.transform.rotation = odom3d_now.pose.pose.orientation;
	tf_broadcaster.sendTransform(transform);
}
