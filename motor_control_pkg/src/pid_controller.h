#include<ros/ros.h>
#include<mutex>
#include<sensor_msgs/JointState.h>
#include<std_msgs/Float64MultiArray.h>
#include"piezo_sonic.h"

#ifndef _INCLUDED_PID_CONTROLLER_H_
#define _INCLUDED_PID_CONTROLLER_H_

#define MOTOR_NUMS 8
#define Fc  1.0

class MotorParameter
{
public:
	// モータ設定用パラメータ
	// 時計回り
	int cw_high_freq;	// CW 高速度用周波数
	int cw_low_freq;	// CW 低速度用周波数
	int cw_phase;		// CW 位相
	// 反時計回り
	int ccw_high_freq;	// CCW 高速度用周波数
	int ccw_low_freq;	// CCW 低速度用周波数
	int ccw_phase;      // CCW 位相
	// 変換関数のパラメータ
	double ap; // 正回転用係数
	double bp; // 正回転用時定数
	double an; // 逆回転用係数
	double bn; // 逆回転用時定数
	
	//
};

class PidController
{
private:
	ros::NodeHandle nh;
	ros::Subscriber joint_states_sub;
	ros::Subscriber next_target_sub;
	ros::Publisher  output_pub;
	ros::Publisher  odom_pub;
	ros::Publisher  tf_pub;

	ros::Time  time_o;
	double now_angle[MOTOR_NUMS];
	double old_angle[MOTOR_NUMS];
	double lock_angle[MOTOR_NUMS];
	double next_target[MOTOR_NUMS];
	double err[MOTOR_NUMS];
	double err_o[MOTOR_NUMS];
	double err_i[MOTOR_NUMS];
	double err_d[MOTOR_NUMS];
	double alpha;
	double x[MOTOR_NUMS];
	int output[MOTOR_NUMS];
	int output_o[MOTOR_NUMS];
	bool get_now_angle;
	bool get_next_target;
	bool first_vel;

	PiezoSonic *motor[MOTOR_NUMS];

	std::mutex m;

	// Parameter
	double Kp[MOTOR_NUMS];
	double Ki[MOTOR_NUMS];
	double Kd[MOTOR_NUMS];
	double wheel_radius;    // 0.05[m]
	double wheel_offset;    // 0.05[m]
	double ticks_per_rotate;    // 4000
	double toRadian;    // =2*PI/ticks_per_rotate
	
	// for Motor
	MotorParameter  para[MOTOR_NUMS];

public:
	PidController();
	~PidController();

	void init();
	void run();
	void joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg);
	void next_target_callback(const std_msgs::Float64MultiArray& msg);
	void publication();
	void cal_pid();
	void drive();
};

#endif
