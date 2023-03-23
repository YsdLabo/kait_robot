#include<ros/ros.h>
#include<nodelet/nodelet.h>
#include<pluginlib/class_list_macros.h>
#include<geometry_msgs/Twist.h>
#include<std_msgs/Int32.h>

#include<driving_controller_pkg/DrivingState.h>
#include<std_srvs/Empty.h>

namespace driving_controller_ns
{

enum class E_STEERING
{
	NONE = -1,
	DIRECTION_STOP = 0,
	DIRECTION_F = 1,
	DIRECTION_B = 2,
	DIRECTION_FL = 3,
	DIRECTION_BR = 4,
	DIRECTION_FR = 5,
	DIRECTION_BL = 6,
	DIRECTION_L = 7,
	DIRECTION_R = 8,
	ROTATION_L = 9,
	ROTATION_R = 10
};
	
class TwistToDriving : public nodelet::Nodelet
{
private:
	ros::NodeHandle nh;
	ros::NodeHandle pnh;
	
	ros::Subscriber sub_cmd_vel;
	ros::Publisher  pub_driving_direction;
	ros::Timer stm_timer;
	
	// Service(Client)
	ros::ServiceClient clientDrivingState;
	ros::ServiceClient clientStoppedState;
	driving_controller_pkg::DrivingState driving_state;
	
	// Parameter
	double MAX_SPEED;
	double MIN_SPEED;
	double MAX_ROTATION;
	double MIN_ROTATION;
	double FREQUENCY;
	
	//
	double course;
	double speed;
	double rotation;
	
	// enum
	enum class E_STATE { NONE, IDLING, STEERING, RUNNING, STOPPING };
	E_STATE main_state = E_STATE::NONE;
	enum class E_ACTION { NONE, ENTRY, DO, EXIT };
	E_ACTION action = E_ACTION::NONE;

	E_STEERING steering_dir_now;
	E_STEERING steering_dir_last;

	static constexpr double AREA_F_L = M_PI / 8.0;			// 22.5
	static constexpr double AREA_L_F = M_PI * 3.0 / 8.0;		// 67.5
	static constexpr double AREA_L_B = M_PI * 5.0 / 8.0;		// 112.5
	static constexpr double AREA_B_L = M_PI * 7.0 / 8.0;		// 157.5
	static constexpr double AREA_F_R = - M_PI / 8.0;		// -22.5
	static constexpr double AREA_R_F = - M_PI * 3.0 / 8.0;		// -67.5
	static constexpr double AREA_R_B = - M_PI * 5.0 / 8.0;		// -112.5
	static constexpr double AREA_B_R = - M_PI * 7.0 / 8.0;		// -157.5
	
public:
	void onInit()
	{
		nh = getNodeHandle();
		pnh = getPrivateNodeHandle();
		
		pnh.param("max_speed", MAX_SPEED, 1.0);
		pnh.param("min_speed", MIN_SPEED, 0.01);
		pnh.param("max_rotation", MAX_ROTATION, 2.0);
		pnh.param("min_rotation", MIN_ROTATION, 0.5);
		pnh.param("frequency", FREQUENCY, 50.0);
		
		sub_cmd_vel = nh.subscribe("cmd_vel", 10, &TwistToDriving::cmd_vel_callback, this);
		pub_driving_direction = nh.advertise<std_msgs::Int32>("driving_direction", 1);
		stm_timer = nh.createTimer(ros::Duration(1.0/FREQUENCY), &TwistToDriving::stm_callback, this);
		
		clientDrivingState = nh.serviceClient<driving_controller_pkg::DrivingState>("DrivingState");
		clientStoppedState = nh.serviceClient<std_srvs::Empty>("StoppedState");
		
		course = 0.0;
		speed = 0.0;
		rotation = 0.0;
	}
	
	~TwistToDriving()
	{
		stop_running();
	}

private:
	void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
	{
		double x = msg->linear.x;
		double y = msg->linear.y;
		course = std::atan2(y, x);
		speed  = std::sqrt(x*x+y*y);
		if(speed > MAX_SPEED) speed = MAX_SPEED;
		if(speed < MIN_SPEED) speed = 0.0;
		if(course > AREA_L_B || course < AREA_R_F) speed *= -1.0;
		
		rotation = msg->angular.z;
		if(std::fabs(rotation) < MIN_ROTATION) rotation = 0.0;
		else speed = (std::sqrt(2) * 0.2 + 0.05) * rotation;
		//NODELET_INFO("%lf:%lf:%lf", course, speed, rotation);
	}

	// Main Loop
	void stm_callback(const ros::TimerEvent& e)
	{
		twist_to_direction();
		
		switch(main_state)
		{
		case E_STATE::NONE:
			main_state = E_STATE::IDLING;
			action = E_ACTION::ENTRY;
			steering_dir_now = E_STEERING::DIRECTION_STOP;
			steering_dir_last = steering_dir_now;
			break;
		case E_STATE::IDLING:
			state_idling();
			break;
		case E_STATE::STEERING:
			state_steering();
			break;
		case E_STATE::RUNNING:
			state_running();
			break;
		case E_STATE::STOPPING:
			state_stopping();
			break;
		}
	}

	void twist_to_direction()
	{
		if(speed_is_zero()) {
			steering_dir_now = E_STEERING::DIRECTION_STOP;
		}
		else {
			if( rotation_is_zero()) {
				if( course_is_in_area_F() ) steering_dir_now = E_STEERING::DIRECTION_F;
				else if( course_is_in_area_L() ) steering_dir_now = E_STEERING::DIRECTION_L;
				else if( course_is_in_area_R() ) steering_dir_now = E_STEERING::DIRECTION_R;
				else if( course_is_in_area_FL() ) steering_dir_now = E_STEERING::DIRECTION_FL;
				else if( course_is_in_area_BL() ) steering_dir_now = E_STEERING::DIRECTION_BL;
				else if( course_is_in_area_FR() ) steering_dir_now = E_STEERING::DIRECTION_FR;
				else if( course_is_in_area_BR() ) steering_dir_now = E_STEERING::DIRECTION_BR;
				else steering_dir_now = E_STEERING::DIRECTION_B;
			}
			else {
				if( course_is_rotate_L() ) steering_dir_now = E_STEERING::ROTATION_L;
				else if( course_is_rotate_R() ) steering_dir_now = E_STEERING::ROTATION_R;
			}
		}
	}
	
	bool course_is_in_area_F()
	{
		return (course >= AREA_F_R && course <= AREA_F_L);  // -22.5〜22.5
	}
	bool course_is_in_area_FL()
	{
		return (course > AREA_F_L && course < AREA_L_F);  // 22.5〜67.5
	}
	bool course_is_in_area_L()
	{
		return (course >= AREA_L_F && course <= AREA_L_B);  // 67.5〜112.5
	}
	bool course_is_in_area_BL()
	{
		return (course > AREA_L_B && course < AREA_B_L);  // 112.5〜157.5
	}
	bool course_is_in_area_FR()
	{
		return (course > AREA_R_F && course < AREA_F_R);  // -67.5〜-22.5
	}
	bool course_is_in_area_R()
	{
		return (course >= AREA_R_B && course <= AREA_R_F);  // -112.5〜-67.5
	}
	bool course_is_in_area_BR()
	{
		return (course > AREA_B_R && course < AREA_R_B);  // -157.5〜-112.5
	}
	bool course_is_rotate_L()
	{
		return (rotation > 0);
	}
	bool course_is_rotate_R()
	{
		return (rotation < 0);
	}
	bool speed_is_zero()
	{
		return (std::fabs(speed) < MIN_SPEED);
	}
	bool rotation_is_zero()
	{
		return (std::fabs(rotation) < MIN_ROTATION);
	}
	void store_current_steering_dir()
	{
		steering_dir_last = steering_dir_now;
	}
	bool course_changed()
	{
		return (steering_dir_now != steering_dir_last);
	}
	bool go_to_stop()
	{
		return (steering_dir_now == E_STEERING::DIRECTION_STOP);
	}
	
	bool check_all_motors_stopped()
	{
		std_srvs::Empty emp;
		bool tmp = clientStoppedState.call(emp);
		return tmp;
	}
	void start_idling()
	{
		driving_state.request.state = 0;
		driving_state.request.steering = 0;
		driving_state.request.speed = 0;
		clientDrivingState.call(driving_state);
	}
	void do_steering()
	{
		int n = static_cast<int>(steering_dir_now);
		driving_state.request.state = 1;
		driving_state.request.steering = (n==0?0:(int)((n-1)/2));
		driving_state.request.speed = 0;
		clientDrivingState.call(driving_state);
	}
	void do_running()
	{
		int n = static_cast<int>(steering_dir_now);
		driving_state.request.state = 2;
		driving_state.request.steering = (n==0?0:(int)((n-1)/2));
		if(rotation_is_zero()) driving_state.request.speed = speed;
		else driving_state.request.speed = rotation * 0.3328;
		clientDrivingState.call(driving_state);
	}
	void stop_running()
	{
		int n = static_cast<int>(steering_dir_now);
		driving_state.request.state = 3;
		driving_state.request.steering = (n==0?0:(int)((n-1)/2));
		driving_state.request.speed = 0;
		clientDrivingState.call(driving_state);
	}
	
	// アイドリング中
	void state_idling()
	{
		if(action == E_ACTION::ENTRY) {
			NODELET_INFO("[State] Idling");
			store_current_steering_dir();
			start_idling();
			action = E_ACTION::DO;
		}
		if(action == E_ACTION::DO) {
			if(course_changed()) {
				main_state = E_STATE::STEERING;
				action = E_ACTION::EXIT;
			}
		}
		if(action == E_ACTION::EXIT) {
			action = E_ACTION::ENTRY;
		}
	}
	
	// 操舵中
	void state_steering()
	{
		static int init_flag = 0;
		if(action == E_ACTION::ENTRY) {
			NODELET_INFO("[State] Steering");
			//if(init_flag == 0) {
				store_current_steering_dir();
				//start_steering();
				//init_flag = 1;
			//}
			//else {
				action = E_ACTION::DO;
				//init_flag = 0;
			//}
			
		}
		if(action == E_ACTION::DO) {
			do_steering();
			if(check_all_motors_stopped()) 
			{
				if(go_to_stop()) main_state = E_STATE::IDLING;
				else if(course_changed()) main_state = E_STATE::STEERING;
				else main_state = E_STATE::RUNNING;
				action = E_ACTION::EXIT;
			}
		}
		if(action == E_ACTION::EXIT) {
			action = E_ACTION::ENTRY;
		}
	}

	// 走行中
	void state_running()
	{
		if(action == E_ACTION::ENTRY) {
			NODELET_INFO("[State] Running");
			// store_current_steering_dir();
			action = E_ACTION::DO;
		}
		if(action == E_ACTION::DO) {
			do_running();
			if(course_changed()) {
				main_state = E_STATE::STOPPING;
				action = E_ACTION::EXIT;
			}
		}
		if(action == E_ACTION::EXIT) {
			action = E_ACTION::ENTRY;
		}
	}

	// 停止中
	void state_stopping()
	{
		if(action == E_ACTION::ENTRY) {
			NODELET_INFO("[State] Stopping");
			stop_running();
			action = E_ACTION::DO;
		}
		if(action == E_ACTION::DO) {
			if(check_all_motors_stopped()) 
			{
				//if(go_to_stop()) main_state = E_STATE::IDLING;
				//else 
				main_state = E_STATE::STEERING;
				action = E_ACTION::EXIT;
			}
		}
		if(action == E_ACTION::EXIT) {
			steering_dir_now = E_STEERING::DIRECTION_STOP;
			action = E_ACTION::ENTRY;
		}
	}
};

}

PLUGINLIB_EXPORT_CLASS(driving_controller_ns::TwistToDriving, nodelet::Nodelet);
