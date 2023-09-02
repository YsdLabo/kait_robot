#include<ros/ros.h>
#include<sensor_msgs/JointState.h>
#include<std_msgs/Int32MultiArray.h>

#define  MOTOR_NUMS  4
#define  VELOCITY_COUNTS  5

std_msgs::Int32MultiArray  enc_data;
sensor_msgs::JointState joint_state;
ros::Publisher  pub;

long enc_count[MOTOR_NUMS] = {0};
double current_angle[MOTOR_NUMS];
double last_angle[MOTOR_NUMS];
double velocity[MOTOR_NUMS][VELOCITY_COUNTS];
constexpr double toRadian = 2.0 * M_PI / 4000.0;
ros::Time current_time;
ros::Time last_time;

void encoderCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
	current_time = ros::Time::now();

	enc_data = *msg;

	enc_count[0] -= enc_data.data[1];    // wheel_fl
	enc_count[1] += enc_data.data[3];    // wheel_fr
	enc_count[2] += enc_data.data[5];    // wheel_br
	enc_count[3] -= enc_data.data[7];    // wheel_bl

	//old
	//enc_count[0] += enc_data.data[0];    // axle_fl
	//enc_count[1] -= enc_data.data[1];    // wheel_fl
	//enc_count[2] += enc_data.data[2];    // axle_fr
	//enc_count[3] += enc_data.data[3];    // wheel_fr
	//enc_count[4] += enc_data.data[4];    // axle_br
	//enc_count[5] += enc_data.data[5];    // wheel_br
	//enc_count[6] += enc_data.data[6];    // axle_bl
	//enc_count[7] -= enc_data.data[7];    // wheel_bl
}

void timerCallback(const ros::TimerEvent& e)
{
	static int num = 0;

	for(int i=0;i<MOTOR_NUMS;i++)
		current_angle[i] = enc_count[i] * toRadian;

	joint_state.header.stamp = current_time;
	joint_state.name.resize(MOTOR_NUMS);
	joint_state.position.resize(MOTOR_NUMS);
	joint_state.velocity.resize(MOTOR_NUMS);

	joint_state.name[0] ="wheel_fl";
	joint_state.name[1] ="wheel_fr";
	joint_state.name[2] ="wheel_br";
	joint_state.name[3] ="wheel_bl";

	// old
	//joint_state.name[0] ="axle_fl";
	//joint_state.name[1] ="wheel_fl";
	//joint_state.name[2] ="axle_fr";
	//joint_state.name[3] ="wheel_fr";
	//joint_state.name[4] ="axle_br";
	//joint_state.name[5] ="wheel_br";
	//joint_state.name[6] ="axle_bl";
	//joint_state.name[7] ="wheel_bl";

	double dt = (current_time - last_time).toSec();
	for(int i=0;i<MOTOR_NUMS;i++) {
		// POSITION
		joint_state.position[i] = current_angle[i];
		// VELOCITY
		velocity[i][num] = (current_angle[i] - last_angle[i]) / dt;
		joint_state.velocity[i] = 0.0;
		for(int j=0;j<VELOCITY_COUNTS;j++) joint_state.velocity[i] += velocity[i][j];
		joint_state.velocity[i] /= (double)VELOCITY_COUNTS;
		last_angle[i] = current_angle[i];
	}
	num++;
	if(num == VELOCITY_COUNTS) num = 0;

	pub.publish(joint_state);
	last_time = current_time;
}

int main(int argc, char** argv)
{
	for(int i=0;i<MOTOR_NUMS;i++) {
		enc_count[i] = 0;
		current_angle[i] = 0;
		last_angle[i] = 0;
	}

	ros::init(argc, argv, "wheel_rotation");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/kait_robot/encoder_count", 10, &encoderCallback);
	pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
	ros::Timer  timer = nh.createTimer(ros::Duration(0.01), &timerCallback);
	last_time = ros::Time::now();

	ros::spin();

	return 0;
}
