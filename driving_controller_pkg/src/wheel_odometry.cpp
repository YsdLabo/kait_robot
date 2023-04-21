#include "wheel_odometry.h"

namespace driving_controller_ns
{

WheelOdometry::WheelOdometry()
{
	cur_x = 0.0;
	cur_y = 0.0;
	cur_th = 0.0;
	
	odom_pub = nh.advertise<nav_msgs::Odometry>("odom2", 1);
}

void WheelOdometry::update(sensor_msgs::JointState& wheel_state)
{
	wheel_state_last = wheel_state;
	first_run = false;
}

void WheelOdometry::update(sensor_msgs::JointState& wheel_state, double steer[4])
{
	if(!first_run) {
		double dv, dth;
		double phi;
		
		double diff_wheel[4];
		for(int i=0;i<4;i++) diff_wheel[i] = wheel_state.position[i] - wheel_state_last.position[i];
		
		// 操舵方向に合わせて，オドメトリ
		// F & B
		if(steer[1] >= areaF_R && steer[1] <= areaF_L)
		{
			// Forward
			if(diff_wheel[0] > 0) dth = (diff_wheel[1] - diff_wheel[0]) / L1;
			// Back
			else dth = (diff_wheel[3] - diff_wheel[2]) / L1;
			dv = (diff_wheel[0]+diff_wheel[1]+diff_wheel[2]+diff_wheel[3]) / 4.0;
			phi = 0.0;
		}
		// FL & BR
		else if(steer[1] > areaF_L && steer[1] < areaL_F && steer[0] < 0.0)
		{
			// Forward Left
			if(diff_wheel[0] > 0) dth = (diff_wheel[1] - diff_wheel[3]) / L2;
			// Backward Right
			else dth = (diff_wheel[3] - diff_wheel[1]) / L2;
			dv = (diff_wheel[0]+diff_wheel[1]+diff_wheel[2]+diff_wheel[3]) / 4.0;
			phi = M_PI / 4.0;
		}
		// FR & BL
		else if(steer[1] > areaR_F && steer[1] < areaF_R)
		{
			// Forward Right
			if(diff_wheel[0] > 0) dth = (diff_wheel[2] - diff_wheel[0]) / L2;
			// Backward Left
			else dth = (diff_wheel[0] - diff_wheel[2]) / L2;
			dv = (diff_wheel[0]+diff_wheel[1]+diff_wheel[2]+diff_wheel[3]) / 4.0;
			phi = - M_PI / 4.0;
		}
		// L & R
		else if(steer[1] >= areaL_F && steer[1] < areaL_B)
		{
			// Left
			if(diff_wheel[0] < 0) dth = (diff_wheel[3] - diff_wheel[0]) / L1;
			// Right
			else dth = (diff_wheel[2] - diff_wheel[1]) / L1;
			dv = (-diff_wheel[0]+diff_wheel[1]+diff_wheel[2]-diff_wheel[3]) / 4.0;
			phi = M_PI / 2.0;
		}
		// Rotation
		else if(steer[1] > areaF_L && steer[1] < areaL_F && steer[0] > 0.0)
		{
			// Rotation
			dth = (-diff_wheel[0]+diff_wheel[1]+diff_wheel[2]-diff_wheel[3]) / 4.0 / L3;
			dv = 0.0;
			phi = 0.0;
		}
		
		cur_th += dth * wheel_radius;
		double dx = dv * std::cos(cur_th + phi) * wheel_radius;
		double dy = dv * std::sin(cur_th + phi) * wheel_radius;
		cur_x += dx;
		cur_y += dy;

		curent_time = wheel_state.header.stamp;
		double dt = (curent_time - wheel_state_last.header.stamp).toSec();
		cur_vx = dx / dt;
		cur_vy = dy / dt;
		cur_w  = dth / dt;
		
		
		publish_odom();
		//publish_tf();
	}
	wheel_state_last = wheel_state;
	first_run = false;
}

void WheelOdometry::publish_odom()
{
	// odom
	odom.header.stamp = curent_time;
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_link";
	
	odom.pose.pose.position.x = cur_x;
	odom.pose.pose.position.y = cur_y;
	odom.pose.pose.position.z = 0;
	odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(cur_th);
	
	odom.pose.covariance[0] = 0.01;
	odom.pose.covariance[7] = 0.01;
	odom.pose.covariance[14] = FLT_MAX;
	odom.pose.covariance[21] = FLT_MAX;
	odom.pose.covariance[28] = FLT_MAX;
	odom.pose.covariance[35] = 0.01;
	
	odom.twist.twist.linear.x = cur_vx;
	odom.twist.twist.linear.y = cur_vy;
	odom.twist.twist.angular.z = cur_w;
	
	odom.twist.covariance[0] = 0.01;
	odom.twist.covariance[7] = 0.01;
	odom.twist.covariance[14] = FLT_MAX;
	odom.twist.covariance[21] = FLT_MAX;
	odom.twist.covariance[28] = FLT_MAX;
	odom.twist.covariance[35] = 0.01;
	
	odom_pub.publish(odom);
}

/*void WheelOdometry::publish_tf()
{
	geometry_msgs::TransformStamped  tf_trans;

	tf_trans.header.stamp = odom.header.stamp;
	tf_trans.header.frame_id = odom.header.frame_id;
	tf_trans.child_frame_id = odom.child_frame_id;
	tf_trans.transform.translation.x = odom.pose.pose.position.x;
	tf_trans.transform.translation.y = odom.pose.pose.position.y;
	tf_trans.transform.translation.z = odom.pose.pose.position.z;
	tf_trans.transform.rotation = odom.pose.pose.orientation;

	try{
		tf_caster.sendTransform(tf_trans);	// error
	}
	catch(...){};
}
*/
}
