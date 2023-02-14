#include<ros/ros.h>
#include<nodelet/nodelet.h>
#include<pluginlib/class_list_macros.h>
#include<geometry_msgs/Twist.h>
#include<std_msgs/Int32.h>

namespace driving_controller
{

class TwistToDriving : public nodelet::Nodelet
{
private:
	ros::NodeHandle nh;
	ros::NodeHandle pnh;
	
	ros::Subscriber sub_cmd_vel;
	ros::Publisher  pub_driving_direction;
	ros::Timer stm_timer;
	
	// Parameter
	double max_speed;
	double frequency;
	
	//
	double course;
	double speed;
	double rotate;
	
	// enum
	enum class E_STATE { NONE, IDLING, STEERING, RUNNING, STOPPING };
	E_STATE main_state = E_STATE::NONE;
	enum class E_ACTION { NONE, ENTRY, DO, EXIT };
	E_ACTION action = E_ACTION::NONE;
	enum class E_STEERING
	{
		NONE,
		DIRECTION_STOP,
		DIRECTION_F,
		DIRECTION_B,
		DIRECTION_L,
		DIRECTION_R,
		DIRECTION_FL,
		DIRECTION_FR,
		DIRECTION_BL,
		DIRECTION_BR,
		ROTATION_L,
		ROTATION_R
	};
	E_STEERING steering_dir = E_STEERING::NONE;
	E_STEERING steering_dir_now;
	E_STEERING stored_steering_dir;
	E_STEERING steering_dir_old;
	
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
		rotate = w;
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
			if( rotate_is_zero() ) {
				if( in_area_F() ) steering_dir_now = E_STEERING::DIRECTION_F;
				else if( in_area_L() ) steering_dir_now = E_STEERING::DIRECTION_L;
				else if( in_area_R() ) steering_dir_now = E_STEERING::DIRECTION_R;
				else if( in_area_FL() ) steering_dir_now = E_STEERING::DIRECTION_FL;
				else if( in_area_BL() ) steering_dir_now = E_STEERING::DIRECTION_BL;
				else if( in_area_FR() ) steering_dir_now = E_STEERING::DIRECTION_FR;
				else if( in_area_BR() ) steering_dir_now = E_STEERING::DIRECTION_BR;
				else steering_dir_now = E_STEERING::DIRECTION_B;
			}
			else {
				if( is_rotate_L() ) steering_dir_now = E_STEERING::ROTATION_L;
				else if( is_rotate_R() ) steering_dir_now = E_STEERING::ROTATION_R;
			}
		}
	}
	
	constexpr double AREA_F_L = M_PI / 8.0;
	constexpr double AREA_F_R = - M_PI / 8.0;
	constexpr double AREA_B_L = M_PI * 7.0 / 8.0;
	constexpr double AREA_B_R = - M_PI * 7.0 / 8.0;
	constexpr double AREA_L_F = M_PI * 3.0 / 8.0;
	constexpr double AREA_L_B = M_PI * 5.0 / 8.0;
	constexpr double AREA_R_F = - M_PI * 3.0 / 8.0;
	constexpr double AREA_R_B = - M_PI * 5.0 / 8.0;
	
	bool in_area_F()
	{
		return (course >= AREA_F_R && course <= AREA_F_L);
	}
};

}
