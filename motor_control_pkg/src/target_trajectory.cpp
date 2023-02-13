#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#define  ABS(x)  (x>=0?x:-(x))
#define  SIGN(x) (x>=0?1:-1)

double angle_d[8] = {0.0};
bool update_angle_d = false;
bool first_update = false;

void callback(const std_msgs::Float64MultiArray& msg)
{
	for(int i=0;i<8;i++) {
		angle_d[i] = msg.data[i];
		ROS_INFO("set angle_d[%d] = %lf", i+1, angle_d[i]);
	}
	update_angle_d = true;
	first_update = true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "target_trajectory");
	ros::NodeHandle nh;
	ros::Subscriber angle_d_sub = nh.subscribe("/kait_robot/angle_d", 10, &callback);
	ros::Publisher  angle_pub = nh.advertise<std_msgs::Float64MultiArray>("/kait_robot/next_target", 1);
	ros::Publisher  velocity_pub = nh.advertise<std_msgs::Float64MultiArray>("/kait_robot/next_velocity", 1);

	ros::NodeHandle pnh("~");

	double delay_time = 1.0;
	pnh.getParam("delay_time", delay_time);
	ROS_INFO("delay_time: %lf", delay_time);

	double a_max = 0.2; // 1.0[m/s^2]
	pnh.getParam("acceleration_max", a_max);
	ROS_INFO("acceleration_max: %lf", a_max);

	double v_max = 0.5; // 1.0[m/s]
	pnh.getParam("velocity_max", v_max);
	ROS_INFO("velocity_max: %lf", v_max);

	double wheel_radius = 0.05;
	pnh.getParam("wheel_radius", wheel_radius);
	ROS_INFO("wheel_radius: %lf", wheel_radius);

	double w_max = v_max;// / wheel_radius;
	double aa_max = a_max;// / wheel_radius;
	ROS_INFO("angular velocity max: %lf", w_max);
	ROS_INFO("angular acceleration max: %lf", aa_max);

	ros::Rate  loop(100);

	double angle[8] = {0.0};
	double vel[8] = {0.0};
	double t0[8] = {-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0};
	double t1[8];  // = w_max / aa_max;
	double t2[8];  // = angle_d / w_max;
	double t3[8];
	double S;
	double x_d[8];
	double angle0[8]={0.0}, angle1[8], angle2[8];
	bool idling[8] = {true, true, true, true, true, true, true, true};

	ros::Time  time_s = ros::Time::now();

	while(ros::ok())
	{
		ros::Time  time_n = ros::Time::now();
		double t = (time_n - time_s).toSec();  // 現在時間	

		if(first_update == true) {
			if(update_angle_d == true) {
				for(int i=0;i<8;i++) {
					double ww;
					if(idling[i] == true) {
						if(ABS(angle_d[0]-angle_d[6])>1.57) ww = w_max / 3.0;
						else ww = w_max;
						S = ww * ww / aa_max;
						x_d[i] = angle_d[i] - angle[i];

						if(ABS(x_d[i]) > S) {
							t1[i] = ww / aa_max;
							t2[i] = ABS(x_d[i]) / ww;
							t3[i] = t1[i] + t2[i];
						}
						else {
							t1[i] = sqrt(ABS(x_d[i]) / aa_max);
							t2[i] = t1[i];
							t3[i] = t1[i] + t2[i];
						}
						angle1[i] = 0.5 * SIGN(x_d[i]) * aa_max * t1[i] * t1[i] + angle0[i];
						angle2[i] = SIGN(x_d[i]) * aa_max * t1[i] * (t2[i] - t1[i]) + angle1[i];

						idling[i] = false;

						t0[i] = t + delay_time;
						t1[i] += t0[i];
						t2[i] += t0[i];
						t3[i] += t0[i];
					}
				}
				update_angle_d = false;
			}

			for(int i=0;i<8;i++) {
				// Phase 0
				if(t <= t0[i]) {
					vel[i] = 0.0;
					//angle[i] = angle0[i];
				}
				// Phase 1
				else if(t > t0[i] && t <= t1[i]) {
					vel[i] = SIGN(x_d[i]) * aa_max * (t - t0[i]);
					angle[i] = 0.5 * vel[i] * (t - t0[i]) + angle0[i];
				}
				// Phase 2
				else if(t > t1[i] && t <= t2[i]) {
					vel[i] = SIGN(x_d[i]) * aa_max * (t1[i] - t0[i]);
					angle[i] = vel[i] * (t - t1[i]) + angle1[i];
				}
				// Phase 3
				else if(t > t2[i] && t <= t3[i]) {
					vel[i] = SIGN(x_d[i]) * aa_max * (t1[i] - t0[i] - t + t2[i]); // aa_max * (t1 - t0) - aa_max * (t - t2);
					angle[i] = (SIGN(x_d[i]) * aa_max * (t1[i] - t0[i]) + vel[i]) * (t - t2[i]) * 0.5 + angle2[i];
				}
				// Phase 4
				else if(t <= t3[i] + 0.5) {
					vel[i] = 0.0;
					angle[i] = angle_d[i];
				}
				else {
					idling[i] = true;
					angle0[i] = angle[i];
				}
			}
		}

//		ROS_INFO("%lf:  %lf : %lf", t, angle, vel);

		std_msgs::Float64MultiArray  angle_msg;
		angle_msg.data.resize(8);
		for(int i=0;i<8;i++) angle_msg.data[i] = angle[i];
		angle_pub.publish(angle_msg);

		std_msgs::Float64MultiArray  vel_msg;
		vel_msg.data.resize(8);
		for(int i=0;i<8;i++) vel_msg.data[i] = vel[i];
		velocity_pub.publish(vel_msg);

		ros::spinOnce();

		loop.sleep();
	}

	return 0;
}
