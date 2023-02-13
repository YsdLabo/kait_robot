#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int64.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include "steering_control.h"

#define M_PI_135    2.35619449019234492885 // 135[deg]
#define M_PI_8      0.39269908169872415480 // 22.5[deg]
#define M_PI_3_8    1.17809724509617246442 // 67.5[deg]
#define M_PI_5_8    1.96349540849362077404 // 112.5[deg]
#define M_PI_7_8    2.74889357189106908365 // 157.5[deg]
#define M_PIx2      6.28318530717958647693 // 360[deg]
#define WHEEL_RADIUS        0.05           //車輪半径[m]
#define AXLE_OFFSET         0.332843       //ロボット中心から車輪の接地点までの距離[m]
#define MAX_RPS             2.0            //最大回転数[rps]_最大(180rpm)_定格(120rpm)
#define MAX_VELOCITY        0.3            // ロボットの最高速度
#define MAX_ROTATION_ANGULAR   M_PIx2      // ロボットの旋回最高速度
#define MIN_ROTATION_ANGULAR   -M_PIx2
//#define MAX_WHEEL_ANGULAR   (M_PIx2 * MAX_RPS) //wheelの最大角速度[m/s]_約12[rad/s]
//#define MIN_WHEEL_ANGULAR   (-M_PIx2 * MAX_RPS) 
#define MAX_WHEEL_ANGULAR  (MAX_VELOCITY / WHEEL_RADIUS)    // 車輪駆動軸の最高回転速度 10rad/s
#define MIN_WHEEL_ANGULAR  (-MAX_VELOCITY / WHEEL_RADIUS)
#define MAX_STEERING_ANGULAR  1.0 // M_PI        // 操舵軸の最高回転速度
#define MIN_STEERING_ANGULAR  -1.0 // -M_PI

#define SIGN(A)  (((A)>0)-((A)<0))

#define MOTOR_NUMS  8
#define MOTOR1  0
#define MOTOR2  1
#define MOTOR3  2
#define MOTOR4  3
#define MOTOR5  4
#define MOTOR6  5
#define MOTOR7  6
#define MOTOR8  7

// STATUS
#define IDLING  0
#define STEERING_START  1
#define STEERING_ACT    2
#define RUNNING_START   3
#define RUNNING_ACT     4
#define RUNNING_STOP    5

// DIRECTION
#define DIRECTION_STOP    0
#define DIRECTION_F       1
#define DIRECTION_B       2
#define DIRECTION_L       3
#define DIRECTION_R       4
#define DIRECTION_FL      5
#define DIRECTION_BR      6
#define DIRECTION_FR      7
#define DIRECTION_BL      8
#define ROTATION_L        9
#define ROTATION_R       10

double speed;
double course;
double angular_vel;
double current_err[MOTOR_NUMS];
double current_angle[MOTOR_NUMS];
double old_angle[MOTOR_NUMS];
double motor_angle_buf[MOTOR_NUMS];
double steering_d[MOTOR_NUMS];
double steering_d_o[MOTOR_NUMS] = {0.0};
double current_angle_s[MOTOR_NUMS];
double current_angle_e[MOTOR_NUMS] = {0.0};

//cmd_velより値を取得
void cmdCallback(const geometry_msgs::Twist& cmd_vel)
{
	double vx = cmd_vel.linear.x;//x方向速度
	double vy = cmd_vel.linear.y;//y方向速度
	double az = cmd_vel.angular.z;//角速度

	if(fabs(vx) < 0.1) vx = 0.0;
	if(fabs(vy) < 0.1) vy = 0.0;
	if(fabs(az) < 0.1) az = 0.0;

	//速度の計算
	//sqrt(x*x + y*y) 車体の移動速度
	//speed モータの角速度
	speed = sqrt(vx*vx + vy*vy) * MAX_WHEEL_ANGULAR; // max=20
	//if(vx<0.0) speed *= -1.0;
	if(speed >= MAX_WHEEL_ANGULAR) speed = MAX_WHEEL_ANGULAR;
	if(speed <= MIN_WHEEL_ANGULAR) speed = MIN_WHEEL_ANGULAR;

	//z 超信地旋回の角速度
	//angular_vel モータの角速度
	angular_vel = az * (AXLE_OFFSET / WHEEL_RADIUS); // 
	if(angular_vel >= MAX_ROTATION_ANGULAR) angular_vel = MAX_ROTATION_ANGULAR;
	if(angular_vel <= MIN_ROTATION_ANGULAR) angular_vel = MIN_ROTATION_ANGULAR;
    
	//進行方向の計算
	course = atan2(vy, vx);
}

void outputCallback(const std_msgs::Float64MultiArray& msg)
{
	for(int i=0;i<MOTOR_NUMS;i++) {
		old_angle[i] = current_angle[i];
		current_angle[i] = msg.data[i+8];
		current_err[i] = msg.data[i+16];
	}

}


int finished_steering = 0;
/*void finishedSteering(const std_msgs::Empty& msgs)
{
	finished_steering = 1;
	ROS_INFO("Check!");
}*/

void finishedSteering(const std_msgs::Int64& servo)
{
	if(servo.data == 1){
		finished_steering = 1;
		ROS_INFO("Steering Finish!");
	}else
		finished_steering = 0;
		
	
}

/*void cmds2Callback(const std_msgs::String& cmds2){
	if(cmds2.data[0] == 'f'){
		steering_d[MOTOR2] = 0.0;
		steering_d[MOTOR4] = 0.0;
		steering_d[MOTOR6] = 0.0;
		steering_d[MOTOR8] = 0.0;
	}else if(cmds2.data[0] == 's'){
		steering_d[MOTOR2] = M_PI;
		steering_d[MOTOR4] = M_PI;
		steering_d[MOTOR6] = -M_PI;
		steering_d[MOTOR8] = -M_PI;
	}else if(cmds2.data[0] == 'l'){             		
		steering_d[MOTOR2] = -M_PI_2;
		steering_d[MOTOR4] = M_PI_2;
		steering_d[MOTOR6] = M_PI_2;
		steering_d[MOTOR8] = -M_PI_2;
	}else if(cmds2.data[0] == 'r'){
		steering_d[MOTOR2] = M_PI_2;
		steering_d[MOTOR4] = -M_PI_2;
		steering_d[MOTOR6] = -M_PI_2;
		steering_d[MOTOR8] = M_PI_2;
	}else if(cmds2.data[0] == 't'){
		steering_d[MOTOR2] = M_PI_2; // d
		steering_d[MOTOR4] = M_PI_2; // a
		steering_d[MOTOR6] = -M_PI_2; // d
		steering_d[MOTOR8] = -M_PI_2;  // a
	}
}*/



int main(int argc, char **argv)
{
    ros::init(argc,argv,"twist_to_targets");
    ros::NodeHandle nh;
    ros::Publisher  next_target_pub = nh.advertise<std_msgs::Float64MultiArray>("/kait_robot/next_target", 1); // angle_d
    ros::Publisher steering_pub = nh.advertise<std_msgs::String>("/kait_robot/cmd_msg",1);
    ros::Publisher control_end_pub = nh.advertise<std_msgs::Empty>("/kait_robot/control_end",1);
    //ros::Subscriber motor_sub = nh.subscribe("/kait_robot/cmd_msg2",10, &cmds2Callback);
    ros::Subscriber cmd_sub = nh.subscribe("/kait_robot/cmd_vel", 1, &cmdCallback);
    ros::Subscriber output_sub = nh.subscribe("/kait_robot/output", 1, &outputCallback);
    ros::Subscriber finished_steering_sub = nh.subscribe("/kait_robot/finished_steering", 1, &finishedSteering);
    
    
    
    ros::Rate loop_rate(100);
    
    std_msgs::String msg;
    std_msgs::Empty end;
    

    std_msgs::Float64MultiArray motor_angle;
    motor_angle.data.resize(MOTOR_NUMS);
	for(int i=0;i<MOTOR_NUMS;i++) motor_angle.data[i] = 0.0;

	int status = IDLING;
	//int finished;
	ROS_INFO("Idling");
	int direction = DIRECTION_STOP, direction_o = DIRECTION_STOP;
	//double steering_d[MOTOR_NUMS];

	double t_c=0.0;
	t_c = ros::Time::now().toSec();

	SteeringControl steering_control[8];
	for(int i=0;i<MOTOR_NUMS;i++) {
		steering_control[i].SetAccMax(1.0);
		steering_control[i].SetDccMax(-5.0);
		steering_control[i].SetVelMax(MAX_STEERING_ANGULAR);
	}

	TrapezoidalVelControl vel_control[MOTOR_NUMS];
	for(int i=0;i<MOTOR_NUMS;i++) {
		vel_control[i].SetAccMax(1.0);
		vel_control[i].SetDccMax(-5.0);
		vel_control[i].SetVelMax(MAX_WHEEL_ANGULAR);
	}
	bool change_direction = false;

	while (ros::ok())
	{
		t_c = ros::Time::now().toSec();
	

		ros::spinOnce();

		if(status == IDLING || status == RUNNING_ACT) {
			if(fabs(speed) < 0.001) {
				if(fabs(angular_vel) < 0.001) {
					direction = DIRECTION_STOP;  // 停止
					//ROS_INFO("direction = STOP");
				}
				else if(angular_vel > 0) {
					direction = ROTATION_L;  // 左旋回
					//ROS_INFO("direction = ROTATION LEFT");
				}
				else if(angular_vel < 0) {
					direction = ROTATION_R;  // 右旋回
					//ROS_INFO("direction = ROTATION RIGHT");
				}
				else direction = direction_o;
			}
			else {   // 並進移動
				// 前後
				if(fabs(course) <= M_PI_8) {
					direction = DIRECTION_F;
					//ROS_INFO("direction = FORWARD");
				}
				else if(fabs(course) >= M_PI_7_8) {
					direction = DIRECTION_B;
					speed *= -1.0;
					//ROS_INFO("direction = BACKWARD");
				}
				// 左右
				else if(course >= M_PI_3_8 && course <= M_PI_5_8) {
					direction = DIRECTION_L;
					//ROS_INFO("direction = LEFT");
				}
				else if(course >= -M_PI_5_8 && course <= -M_PI_3_8) {
					direction = DIRECTION_R;
					//ROS_INFO("direction = RIGHT");
				}
				// 左前・右後
				else if(course > M_PI_8 && course < M_PI_3_8) {
					direction = DIRECTION_FL;
					//ROS_INFO("direction = FRONT LEFT");
				} 
				else if(course > -M_PI_7_8 && course < -M_PI_5_8) {
					direction = DIRECTION_BR;
					speed *= -1.0;
					//ROS_INFO("direction = BACK RIGHT");
				}
				// 右前・左後
				else if(course > M_PI_5_8 && course < M_PI_7_8) {
					direction = DIRECTION_BL;
					speed *= -1.0;
					//ROS_INFO("direction = BACK LEFT");
				}
				else if(course > -M_PI_3_8 && course < -M_PI_8) {
					direction = DIRECTION_FR;
					//ROS_INFO("direction = FRONT RIGHT");
				}
				else direction = direction_o;
			}
			change_direction = direction-direction_o==0?false:true;
			direction_o = direction;
		}
		//ROS_INFO("s=%lf, c=%lf , d=%d", speed, course, direction);

		// Idling
		if(status == IDLING) {
		//ROS_INFO("Idling");
			if(change_direction) {
				// 停止・前後(0,1,2)
				if(direction <= DIRECTION_B) {
					/*steering_d[MOTOR1] = 0.0;
					steering_d[MOTOR3] = 0.0;
					steering_d[MOTOR5] = 0.0;
					steering_d[MOTOR7] = 0.0;*/
					msg.data="f";
					ROS_INFO("Front,Back");
					//steering_pub.publish(msg);
					//if(cmds2.data[0] = 'f'){
					steering_d[MOTOR2] = 0.0;
					steering_d[MOTOR4] = 0.0;
					steering_d[MOTOR6] = 0.0;
					steering_d[MOTOR8] = 0.0;
					//}
				}
				// 左右(3,4)
				else if(direction <= DIRECTION_R) {
					/*steering_d[MOTOR1] = -M_PI_2; // n
					steering_d[MOTOR3] = M_PI_2;  // p
					steering_d[MOTOR5] = -M_PI_2; // n
					steering_d[MOTOR7] = M_PI_2;  // p*/
					msg.data="s";
					ROS_INFO("SIDE");
					//steering_pub.publish(msg);
					//if(cmds2.data[0] = 's'){
					steering_d[MOTOR2] = M_PI;
					steering_d[MOTOR4] = M_PI;
					steering_d[MOTOR6] = -M_PI;
					steering_d[MOTOR8] = -M_PI;
					//}
				}
				// 左前・右後(5,6)
				else if(direction <= DIRECTION_BR) {
					/*steering_d[MOTOR1] = M_PI_4;  // p
					steering_d[MOTOR3] = M_PI_4;  // p
					steering_d[MOTOR5] = M_PI_4;  // p
					steering_d[MOTOR7] = M_PI_4;  // p*/
					msg.data="l";
					ROS_INFO("L-Front,R-Rear");
					//steering_pub.publish(msg); 
					//if(cmds2.data[0] = 'l'){             		
					steering_d[MOTOR2] = -M_PI_2;
					steering_d[MOTOR4] = M_PI_2;
					steering_d[MOTOR6] = M_PI_2;
					steering_d[MOTOR8] = -M_PI_2;
					//}
				}
				// 右前・左後(7,8)
				else if(direction <= DIRECTION_BL) {
					/*steering_d[MOTOR1] = -M_PI_4;  // n
					steering_d[MOTOR3] = -M_PI_4;  // n
					steering_d[MOTOR5] = -M_PI_4;  // n
					steering_d[MOTOR7] = -M_PI_4;  // n*/
					msg.data="r";
					ROS_INFO("R-Front,L-Rear");
					//steering_pub.publish(msg);
                	//if(cmds2.data[0] = 'r'){
					steering_d[MOTOR2] = M_PI_2;
					steering_d[MOTOR4] = -M_PI_2;
					steering_d[MOTOR6] = -M_PI_2;
					steering_d[MOTOR8] = M_PI_2;
					//}
                }
				// 旋回(9,10)
				else if(direction <= ROTATION_R){ //　旋回
					/*steering_d[MOTOR1] = -M_PI_4;  // n
					steering_d[MOTOR3] = M_PI_4;   // p
					steering_d[MOTOR5] = -M_PI_4;  // n
					steering_d[MOTOR7] = M_PI_4;   // p*/
					msg.data="t";
					ROS_INFO("Turn");
					//steering_pub.publish(msg);
                	//if(cmds2.data[0] = 't'){
					steering_d[MOTOR2] = M_PI_2; // d
					steering_d[MOTOR4] = M_PI_2; // a
					steering_d[MOTOR6] = -M_PI_2; // d
					steering_d[MOTOR8] = -M_PI_2;  // a
					//}
            	}
				
				status = STEERING_START;
				ROS_INFO("Steering");
			}
			next_target_pub.publish(motor_angle);
			//for(int i=0;i<MOTOR_NUMS;i++)
			//ROS_INFO("motor_angle.data[%d] : %.4f",i,motor_angle.data[i]);
			//ROS_INFO("==================");
		}
		// Steering
		else if(status == STEERING_START) {
			steering_pub.publish(msg);

			for(int i=0;i<MOTOR_NUMS;i++) {
				if(i%2==0) {
					if(fabs(fabs(steering_d[MOTOR1]-motor_angle.data[MOTOR1])-fabs(steering_d[MOTOR7]-motor_angle.data[MOTOR7])) > 1.0) steering_control[i].SetVelMax(MAX_STEERING_ANGULAR / 3.0);
					else steering_control[i].SetVelMax(MAX_STEERING_ANGULAR);
					steering_control[i].Start(steering_d[i], motor_angle.data[i]);
				}
				else steering_control[i].Start(steering_d[i], current_angle[i]);
			}
			for(int i=1;i<MOTOR_NUMS;i+=2)
				current_angle_e[i] = current_angle[i];
							
			finished_steering = 0;
			
			status = STEERING_ACT;
		}
		else if(status == STEERING_ACT) {
			
			int finished = 0;

			// 車軸の台形制御が終わっているかどうか
			for(int i=1;i<MOTOR_NUMS;i+=2) {
				motor_angle.data[i] = steering_control[i].Next(t_c);
				//if(steering_control[i].Finished()) {
				//	finished ++;
				//}
				double buf = fabs((current_angle_e[i] + (steering_d[i] - steering_d_o[i])) - current_angle[i]);
				printf("%lf : ", buf);
				if(buf < 0.067) finished ++;
			}
			printf("\n");
			
			ROS_INFO("finished (%f : %f) %d", steering_d[7], current_angle[7], finished);
			
			//ROS_INFO("Motor_Angle Finished!");
			
			control_end_pub.publish(end);	//KHR_TEST4.CPPにPUBLISH
			
			next_target_pub.publish(motor_angle);
			// KRS servo
			steering_pub.publish(msg);

			// 操舵角が目標に達したかどうかをチェック
			//double sum_err = 0.0;
			//for(int i=0;i<MOTOR_NUMS;i+=2) {
			//	sum_err += fabs(current_err[i]);
			//}
			
			// 台形制御が終わり、操舵角が目標に達していたら、次の状態に進む
			if(finished == 4 && finished_steering == 1) {
				//int finished = 0;
				//finished_steering = 0;
				//for(int i=1;i<MOTOR_NUMS;i+=2) {
				//	motor_angle.data[i] = 0.0; //vel_control[i].Next(t_c);
					//if(vel_control[i].Finished()) finished ++;
				//}
				for(int i=1;i<MOTOR_NUMS;i+=2) {
					current_angle_s[i] = current_angle[i];
					steering_d_o[i] = steering_d[i];
				}
				
				if(direction == DIRECTION_STOP) {
					status = IDLING;
					ROS_INFO("Steering -> Idling");
					ROS_INFO("-------------");
				}
				else {
					status = RUNNING_START;
					ROS_INFO("Running");
				}
			}

		}
		// Running
		else if(status == RUNNING_START) {
		
			/*// 左右
			if(direction == DIRECTION_L || direction == DIRECTION_R) {
				vel_control[1].Init(- SIGN(course)*speed, motor_angle.data[1]);
				vel_control[3].Init( SIGN(course)*speed, motor_angle.data[3]);
				vel_control[5].Init(- SIGN(course)*speed, motor_angle.data[5]);
				vel_control[7].Init( SIGN(course)*speed, motor_angle.data[7]);
			}
			// 旋回
			else if(direction == ROTATION_L || direction == ROTATION_R) {
				vel_control[1].Init(-angular_vel, motor_angle.data[1]);
				vel_control[3].Init( angular_vel, motor_angle.data[3]);
				vel_control[5].Init( angular_vel, motor_angle.data[5]);
				vel_control[7].Init(-angular_vel, motor_angle.data[7]);
			}
			// その他
			else {
				for(int i=1;i<MOTOR_NUMS;i+=2)
					vel_control[i].Init(speed, motor_angle.data[i]);  // 駆動軸
			}
			*/
			for(int i=0;i<MOTOR_NUMS;i+=2) {
				motor_angle_buf[i] = motor_angle.data[i];
				motor_angle.data[i] = 99;
			}
			
			status = RUNNING_ACT;
		}
		else if(status == RUNNING_ACT) {
		/*
			if(direction == DIRECTION_L || direction == DIRECTION_R) {
				vel_control[1].SetVel(- SIGN(course)*speed);
				vel_control[3].SetVel(SIGN(course)*speed);
				vel_control[5].SetVel(- SIGN(course)*speed);
				vel_control[7].SetVel(SIGN(course)*speed);
			}
			else if(direction == ROTATION_L || direction == ROTATION_R) {
				vel_control[1].SetVel(-angular_vel);
				vel_control[3].SetVel( angular_vel);
				vel_control[5].SetVel( angular_vel);
				vel_control[7].SetVel(-angular_vel);
			}
			else {
				for(int i=1;i<MOTOR_NUMS;i+=2) vel_control[i].SetVel(speed);
			}
			
			for(int i=1;i<MOTOR_NUMS;i+=2) {
				motor_angle.data[i] = vel_control[i].Next(t_c);
			}
			*/
			if(direction == DIRECTION_L || direction == DIRECTION_R) {
				motor_angle.data[1] = - SIGN(course)*speed;
				motor_angle.data[3] = SIGN(course)*speed;
				motor_angle.data[5] = - SIGN(course)*speed;
				motor_angle.data[7] = SIGN(course)*speed;
			}
			else if(direction == ROTATION_L || direction == ROTATION_R) {
				motor_angle.data[1] = - angular_vel;
				motor_angle.data[3] = angular_vel;
				motor_angle.data[5] = angular_vel;
				motor_angle.data[7] = - angular_vel;
			}
			else {
				for(int i=1;i<MOTOR_NUMS;i+=2) motor_angle.data[i] = speed;
			}
			
			next_target_pub.publish(motor_angle);

			if(change_direction == true) {
				for(int i=1;i<MOTOR_NUMS;i+=2) {
					//vel_control[i].SetVel(0.0);
					motor_angle.data[i] = 0.0;
				}
				status = RUNNING_STOP;
				ROS_INFO("STOP NOW");
			}
			
        }
		else if(status == RUNNING_STOP) {
			int finished = 0;
			for(int i=1;i<MOTOR_NUMS;i+=2) {
				motor_angle.data[i] = 0.0; //vel_control[i].Next(t_c);
				//if(vel_control[i].Finished()) finished ++;
			}
			next_target_pub.publish(motor_angle);
			double sum_err = 0.0;
			for(int i=1;i<MOTOR_NUMS;i+=2) {
				sum_err += fabs(current_angle[i] - old_angle[i]); //fabs(current_err[i]);  // 停止を確認
			}
			//next_target_pub.publish(motor_angle);
			//ROS_INFO("STOPPING  --  err = %lf", sum_err);
			if(sum_err<0.01) {
				ROS_INFO("STOPPED");
				status = IDLING;
				ROS_INFO("Running -> Idling");
				ROS_INFO("-------------");
				for(int i=0;i<MOTOR_NUMS;i+=2) {
					motor_angle.data[i] = motor_angle_buf[i];
					//for(int i=0;i<MOTOR_NUMS;i++)
					//ROS_INFO("motor_angle.data[%d] : %.4f",i,motor_angle.data[i]);
					//ROS_INFO("==================");
				}

				change_direction = false;
				direction_o = -1;
				ros::Duration(0.1).sleep();
				//control_end_pub.publish(end);
			}
		}

		loop_rate.sleep();
	}
	return 0;
}


