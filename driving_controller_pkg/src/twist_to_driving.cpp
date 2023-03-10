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
	NONE,
	DIRECTION_STOP = 0,
	DIRECTION_F = 0,
	DIRECTION_B = 0,
	DIRECTION_FL = 1,
	DIRECTION_BR = 1,
	DIRECTION_FR = 2,
	DIRECTION_BL = 2,
	DIRECTION_L = 3,
	DIRECTION_R = 3,
	ROTATION_L = 4,
	ROTATION_R = 4
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
	double max_speed;
	double frequency;
	
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
	E_STEERING stored_steering_dir;
	
public:
	void onInit()
	{
		nh = getNodeHandle();
		pnh = getPrivateNodeHandle();
		
		pnh.param("max_speed", max_speed, "1.0");
		pnh.param("frequency", frequency, "50");
		
		sub_cmd_vel = nh.subscribe("~cmd_vel", 10, &TwistToDriving::cmd_vel_callback, this);
		pub_driving_direction = nh.advertise<std_msgs::Int32>("driving_direction", 1);
		stm_timer = nh.createTimer(ros::Duration(1.0/frequency), &TwistToDriving::stm_callback, this);
		
		clientDrivingState = nh.serviceClient<driving_controller_pkg::DrivingState>("DrivingState");
		clientStoppedState = nh.serviceClient<std_srvs::Empty>("StoppedState");
		driving_state.request.state = static_cast<int>(E_STATE::NONE);
	}
	
private:
	void cmd_vel_callback(const geometry_msgs::Twist& msg)
	{
		double x = msg->linear.x;
		double y = msg->linear.y;
		double w = msg->angular.z;
		
		course = std::atan2(y, x);
		speed  = std::sqrt(x*x+y*y);
		if(speed > MAX_SPEED) speed = MAX_SPEED;
		rotation = w;
	}
	
	void stm_callback(const ros::TimerEvent& e)
	{
		twist_to_direction();
		switch(main_state)
		{
		case E_STATE::NONE:
			main_state = E_STATE::IDLING;
			action = E_ACTION::ENTRY;
			steering_dir_now = E_STEERING::DIRECTION_STOP;
			store_steering_dir = steering_dir_now;
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
		if(!speed_is_zero()) {
			steering_dir_now = E_STEERING::DIRECTION_STOP;
		}
		else {
			if( rotation_is_zero() ) {
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
				if( is_rotate_L() ) steering_dir_now = E_STEERING::ROTATION_L;
				else if( is_rotate_R() ) steering_dir_now = E_STEERING::ROTATION_R;
			}
		}
	}
	
	constexpr double AREA_F_L = M_PI / 8.0;			// 22.5
	constexpr double AREA_L_F = M_PI * 3.0 / 8.0;		// 67.5
	constexpr double AREA_L_B = M_PI * 5.0 / 8.0;		// 112.5
	constexpr double AREA_B_L = M_PI * 7.0 / 8.0;		// 157.5
	constexpr double AREA_F_R = - M_PI / 8.0;		// -22.5
	constexpr double AREA_R_F = - M_PI * 3.0 / 8.0;		// -67.5
	constexpr double AREA_R_B = - M_PI * 5.0 / 8.0;		// -112.5
	constexpr double AREA_B_R = - M_PI * 7.0 / 8.0;		// -157.5
	
	bool course_is_in_area_F()
	{
		return (course >= AREA_F_R && course <= AREA_F_L);  // -22.5???22.5
	}
	bool course_is_in_area_FL()
	{
		return (course > AREA_F_L && course < AREA_L_F);  // 22.5???67.5
	}
	bool course_is_in_area_L()
	{
		return (course >= AREA_L_F && course <= AREA_L_B);  // 67.5???112.5
	}
	bool course_is_in_area_BL()
	{
		return (course > AREA_L_B && course < AREA_B_L);  // 112.5???157.5
	}
	bool course_is_in_area_FR()
	{
		return (course > AREA_R_F && course < AREA_F_R);  // -67.5???-22.5
	}
	bool course_is_in_area_R()
	{
		return (course >= AREA_R_B && course <= AREA_R_F);  // -112.5???-67.5
	}
	bool course_is_in_area_BR()
	{
		return (course > AREA_B_R && course < AREA_R_B);  // -157.5???-112.5
	}
	bool speed_is_zero()
	{
		return (speed < 0.01);
	}
	bool rotation_is_zero()
	{
		return (rotation < 0.5);
	}
	void store_current_steering_dir()
	{
		stored_steering_dir = steering_dir_now;
	}
	bool course_changed()
	{
		return (steering_dir_now != stored_steering_dir);
	}
	
	bool check_all_motors_stopped()
	{
		std_srvs::Empty emp;
		return clientStoppedState.call(emp);
		//return motor_controller.check_stopped_motors();
	}
	void start_steering()
	{
		driving_state.request.state = static_cast<int>(steering_dir_now);
		driving_state.request.speed = 0;
		clientDrivingState.call(driving_state);
		//motor_controller.steer(static_cast<double>(steering_dir_now));
	}
	void start_running()
	{
		driving_state.request.state = static_cast<int>(steering_dir_now);
		driving_state.request.speed = speed;
		clientDrivingState.call(driving_state);
		//motor_controller.move(static_cast<double>(steering_dir_now), speed);
	}
	void stop_running()
	{
		driving_state.request.state = static_cast<int>(steering_dir_now);
		driving_state.request.speed = 0;
		clientDrivingState.call(driving_state);
		//motor_controller.stop();
	}
	
	// ?????????????????????
	void state_idling()
	{
		if(action == E_ACTION::ENTRY) {
			store_current_steering_dir();
			action = E_ACTION::DO;
		}
		if(action == E_ACTION::DO) {
			if( course_changed() ) {
				main_state = E_STATE::STEERING;
				action = E_ACTION::EXIT;
			}
		}
		if(action == E_ACTION::EXIT) {
			action = E_ACTION::ENTRY;
		}
	}
	
	// ?????????
	void state_steering()
	{
		if(action == E_ACTION::ENTRY) {
			store_current_steering_dir();
			start_steering();
			action = E_ACTION::DO;
		}
		if(action == E_ACTION::DO) {
			if(check_all_motors_stopped()) {
				if(course_changed()) main_state = E_STATE::STEERING;
				else main_state = E_STATE::RUNNING;
				action = E_ACTION::EXIT;
			}
		}
		if(action == E_ACTION::EXIT) {
			action = E_ACTION::ENTRY;
		}
	}

	// ?????????
	void state_running()
	{
		if(action == E_ACTION::ENTRY) {
			// store_current_steering_dir();
			action = E_ACTION::DO;
		}
		if(action == E_ACTION::DO) {
			start_running();
			if(course_changed()) {
				main_state = E_STATE::STOPPING;
				action = E_ACTION::EXIT;
			}
		}
		if(action == E_ACTION::EXIT) {
			action = E_ACTION::ENTRY;
		}
	}

	// ?????????
	void state_stopping()
	{
		if(action == E_ACTION::ENTRY) {
			stop_running();
			action = E_ACTION::DO;
		}
		if(action == E_ACTION::DO) {
			if(check_all_motors_stopped()) {
				main_state = E_STATE::IDLING;
				action = E_ACTION::EXIT;
			}
		}
		if(action == E_ACTION::EXIT) {
			action = E_ACTION::ENTRY;
		}
	}

};

}

PLUGINLIB_EXPORT_CLASS(driving_controller_ns::TwistToDriving, nodelet::Nodelet)
