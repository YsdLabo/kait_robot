#include "wheel_odometry.h"

namespace driving_controller_ns
{

WheelOdometry::WheelOdometry()
{
	cur_x = 0.0;
	cur_y = 0.0;
	cur_th = 0.0;
	
	odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1);
}

void WheelOdometry::update(sensor_msgs::JointState& wheel_state)
{
	current_time = wheel_state.header.stamp;
	if(current_time == wheel_state_last.header.stamp) {
		ROS_INFO("[Odometry update1] Ooops! current_time == last_time");
		return;
	}
	publish_odom();
	wheel_state_last = wheel_state;
	first_run = false;
}

void WheelOdometry::update(sensor_msgs::JointState& wheel_state, double steer[4])
{
	if(!first_run) {
		double dv, dth;
		double phi;
		
		current_time = wheel_state.header.stamp;
		if(current_time == wheel_state_last.header.stamp) {
			ROS_INFO("[Odometry update2] Ooops! current_time == last_time");
			return;
		}

		double diff_wheel[4];
		for(int i=0;i<4;i++) diff_wheel[i] = wheel_state.position[i] - wheel_state_last.position[i];
		
		// 操舵方向に合わせて，オドメトリ
		// F & B
		if(steer[1] >= AREA_F_R && steer[1] <= AREA_F_L)
		{
			// Forward
			if(diff_wheel[0] > 0) dth = wheel_radius * (diff_wheel[1] - diff_wheel[0]) / L1;
			// Back
			else dth = wheel_radius * (diff_wheel[3] - diff_wheel[2]) / L1;
			dv = wheel_radius * (diff_wheel[0]+diff_wheel[1]+diff_wheel[2]+diff_wheel[3]) / 4.0;
			phi = 0.0;
		}
		// FL & BR
		else if(steer[1] > AREA_F_L && steer[1] < AREA_L_F && steer[0] > 0.0)
		{
			// Forward Left & Backward Right
			dth = wheel_radius * (diff_wheel[1] - diff_wheel[3]) / L2;
			dv = wheel_radius * (diff_wheel[0]+diff_wheel[1]+diff_wheel[2]+diff_wheel[3]) / 4.0;
			phi = M_PI / 4.0;
		}
		// FR & BL
		else if(steer[1] > AREA_R_F && steer[1] < AREA_F_R)
		{
			// Forward Right & Backward Left
			dth = wheel_radius * (diff_wheel[2] - diff_wheel[0]) / L2;
			dv = wheel_radius * (diff_wheel[0]+diff_wheel[1]+diff_wheel[2]+diff_wheel[3]) / 4.0;
			phi = - M_PI / 4.0;
		}
		// L & R
		else if(steer[1] >= AREA_L_F && steer[1] < AREA_L_B)
		{
			// Left
			if(diff_wheel[0] < 0) dth = wheel_radius * (-diff_wheel[0] - diff_wheel[3]) / L1;
			// Right
			else dth = wheel_radius * (-diff_wheel[2] - diff_wheel[1]) / L1;
			dv = wheel_radius * (-diff_wheel[0]+diff_wheel[1]-diff_wheel[2]+diff_wheel[3]) / 4.0;
			phi = M_PI / 2.0;
		}
		// Rotation
		else if(steer[1] > AREA_F_L && steer[1] < AREA_L_F && steer[0] < 0.0)
		{
			// Rotation
			//dth = wheel_radius * (-diff_wheel[0]+diff_wheel[1]+diff_wheel[2]-diff_wheel[3]) / 4.0 / L3;
			dth = wheel_radius * (-diff_wheel[0]) / L3;
			dv = 0.0;
			phi = 0.0;
		}
		
		double dx, dy;
		if(std::fabs(dth) < 0.0001) {
			double th = cur_th + phi;
			dx = dv * std::cos(th);
			dy = dv * std::sin(th);
		}
		else {
			double R = dv / dth;
			double dL = 2.0 * R * std::sin(dth / 2.0);
			double th = cur_th + dth / 2.0 + phi;
			dx = dL * std::cos(th);
			dy = dL * std::sin(th);
		}
		cur_x += dx;
		cur_y += dy;
		cur_th += dth;
		while(cur_th > M_PI) cur_th -= 2.0 * M_PI;
		while(cur_th < - M_PI) cur_th += 2.0 * M_PI;
		printf("th : %lf :: %lf: %lf: %lf: %lf\n", cur_th, wheel_state.position[0], wheel_state.position[1], wheel_state.position[2], wheel_state.position[3]);

		double dt = (current_time - wheel_state_last.header.stamp).toSec();
		cur_vx = dx / dt;
		cur_vy = dy / dt;
		cur_w  = dth / dt;
		
		publish_odom();
	}
	wheel_state_last = wheel_state;
	first_run = false;
}

void WheelOdometry::publish_odom()
{
	// odom
	odom.header.stamp = current_time;
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
	publish_tf();
}

void WheelOdometry::publish_tf()
{
	geometry_msgs::TransformStamped  tf_trans;

	tf_trans.header.stamp = odom.header.stamp;
	tf_trans.header.frame_id = odom.header.frame_id;
	tf_trans.child_frame_id = odom.child_frame_id;
	tf_trans.transform.translation.x = odom.pose.pose.position.x;
	tf_trans.transform.translation.y = odom.pose.pose.position.y;
	tf_trans.transform.translation.z = odom.pose.pose.position.z;
	tf_trans.transform.rotation = odom.pose.pose.orientation;

	//try{
		tf_caster.sendTransform(tf_trans);
	//}
	//catch(...){};
}

}
