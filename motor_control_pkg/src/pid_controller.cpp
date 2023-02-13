#include"pid_controller.h"

PidController::PidController()
{
	init();
}

PidController::~PidController()
{
	for (int i = 0; i < 8; i++) motor[i]->close();
	ROS_INFO("usb_exit()");
}

void PidController::init()
{

	ROS_INFO("initialize");
	bool res;
	ros::NodeHandle pnh("~");

	XmlRpc::XmlRpcValue pid_list;
	res = pnh.getParam("pid_param", pid_list);
	if (res == false) ROS_INFO("use default value: pid_param");

	for (int i = 0; i < MOTOR_NUMS; i++) {
		Kp[i] = 0.0;
		Ki[i] = 0.0;
		Kd[i] = 0.0;
	}

	if (res == true && pid_list.size() <= MOTOR_NUMS) {
		for (int i = 0; i < pid_list.size(); i++) {
			if (pid_list[i]["number"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
				int n = static_cast<int>(pid_list[i]["number"]);
				if (n >= 1 && n <= 8) {
					ROS_INFO("number %d:", n);
					if (pid_list[i]["Kp"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
						Kp[n - 1] = static_cast<double>(pid_list[i]["Kp"]);
						ROS_INFO("Kp: %lf", Kp[n-1]);
					}
					else {
						ROS_ERROR("invalid value : Kp");
						exit(-1);
					}

					if (pid_list[i]["Ki"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
						Ki[n - 1] = static_cast<double>(pid_list[i]["Ki"]);
						ROS_INFO("Ki: %lf", Ki[n-1]);
					}
					else {
						ROS_ERROR("invalid value : Ki");
						exit(-1);
					}

					if (pid_list[i]["Kd"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
						Kd[n - 1] = static_cast<double>(pid_list[i]["Kd"]);
						ROS_INFO("Kd: %lf", Kd[n-1]);
					}
					else {
						ROS_ERROR("invalid value : Kd");
						exit(-1);
					}
				}
				else {
					ROS_ERROR("out of range : number [1-8]");
					exit(-1);
				}
			}
			else {
				ROS_ERROR("invalue value : number");
				exit(-1);
			}
		}
	}
	else {
		ROS_ERROR("too many parameters");
		exit(-1);
	}

	wheel_radius = 0.05;
	res = pnh.getParam("wheel_radius", wheel_radius);
	if (res == false) ROS_INFO("use default value : wheel_radius");
	ROS_INFO("wheel_radius: %lf", wheel_radius);

	wheel_offset = 0.05;
	pnh.getParam("wheel_offset", wheel_offset);
	if (res == false) ROS_INFO("use default value : wheel_offset");
	ROS_INFO("wheel_offset: %lf", wheel_offset);

	ticks_per_rotate = 4000.0;
	pnh.getParam("ticks_per_rotate", ticks_per_rotate);
	if (res == false) ROS_INFO("use default value : ticks_per_rotate");
	ROS_INFO("ticks_per_rotate: %lf", ticks_per_rotate);
	toRadian = 2.0 * M_PI / ticks_per_rotate;

	// Motor Parameter
	para[0].cw_high_freq = 0x2B00;
	para[0].cw_low_freq = 0x2F00;
	para[0].cw_phase = 0x2400;
	para[0].ccw_high_freq = 0x2B00;
	para[0].ccw_low_freq = 0x2F00;
	para[0].ccw_phase = 0x2400;
	para[0].ap = 0;
	para[0].bp = 0;
	para[0].an = 0;
	para[0].bn = 0;


	para[1].cw_high_freq = 0x2905;
	para[1].cw_low_freq = 0x2D07;
	para[1].cw_phase = 0x2400;
	para[1].ccw_high_freq = 0x2905;
	para[1].ccw_low_freq = 0x2E01;
	para[1].ccw_phase = 0x2400;
	para[1].ap = 3500.0;
	para[1].bp = 5.0;
	para[1].an = -3500.0;
	para[1].bn = -4.6;

	para[2].cw_high_freq = 0x2B00;
	para[2].cw_low_freq = 0x2F00;
	para[2].cw_phase = 0x2400;
	para[2].ccw_high_freq = 0x2B00;
	para[2].ccw_low_freq = 0x2F00;
	para[2].ccw_phase = 0x2400;
	para[2].ap = 0;
	para[2].bp = 0;
	para[2].an = 0;
	para[2].bn = 0;

	para[3].cw_high_freq = 0x2A01;
	para[3].cw_low_freq = 0x2F04;
	para[3].cw_phase = 0x2400;
	para[3].ccw_high_freq = 0x2A02;
	para[3].ccw_low_freq = 0x2F04;
	para[3].ccw_phase = 0x2400;
	para[3].ap = 3500.0;
	para[3].bp = 5.0;	//値を上げてもいいかも
	para[3].an = -3500.0;
	para[3].bn = -4.2;	//値を上げてもいいかも

	para[4].cw_high_freq = 0x2B00;
	para[4].cw_low_freq = 0x2F00;
	para[4].cw_phase = 0x2400;
	para[4].ccw_high_freq = 0x2B00;
	para[4].ccw_low_freq = 0x2F00;
	para[4].ccw_phase = 0x2400;
	para[4].ap = 0;
	para[4].bp = 0;
	para[4].an = 0;
	para[4].bn = 0;

	para[5].cw_high_freq = 0x2903;
	para[5].cw_low_freq = 0x2E00;
	para[5].cw_phase = 0x2400;
	para[5].ccw_high_freq = 0x2903;
	para[5].ccw_low_freq = 0x2E03;
	para[5].ccw_phase = 0x2400;
	para[5].ap = 3500.0;
	para[5].bp = 4.2;
	para[5].an = -3500.0;
	para[5].bn = -4.2;

	para[6].cw_high_freq = 0x2B00;
	para[6].cw_low_freq = 0x2F00;
	para[6].cw_phase = 0x2400;
	para[6].ccw_high_freq = 0x2B00;
	para[6].ccw_low_freq = 0x2F00;
	para[6].ccw_phase = 0x2400;
	para[6].ap = 0;
	para[6].bp = 0;
	para[6].an = 0;
	para[6].bn = 0;

	para[7].cw_high_freq = 0x2902;
	para[7].cw_low_freq =  0x2E04;
	para[7].cw_phase = 0x2400;
	para[7].ccw_high_freq = 0x2902;
	para[7].ccw_low_freq = 0x2D07;
	para[7].ccw_phase = 0x2400;
	para[7].ap = 3500.0;
	para[7].bp = 4.8;
	para[7].an = -3500.0;
	para[7].bn = -4.5;
	
	//parce[0]

	for(int i=0; i<MOTOR_NUMS; i++) {
		motor[i] = new PiezoSonic(i);
		motor[i]->open();
		motor[i]->config(0x00, para[i].cw_high_freq);
		motor[i]->config(0x02, para[i].cw_low_freq);
		motor[i]->config(0x04, para[i].cw_phase);
		motor[i]->config(0x08, para[i].ccw_high_freq);
		motor[i]->config(0x0A, para[i].ccw_low_freq);
		motor[i]->config(0x0C, para[i].ccw_phase);
	}

	alpha = 1.0 / (1.0 + 2.0 * M_PI * Fc * 0.01);

	get_now_angle = false;
	get_next_target = false;
	first_vel = true;
}

void PidController::run()
{
	joint_states_sub = nh.subscribe("/kait_robot/joint_states", 1, &PidController::joint_states_callback, this);
	next_target_sub = nh.subscribe("/kait_robot/next_target", 1, &PidController::next_target_callback, this);
	output_pub = nh.advertise<std_msgs::Float64MultiArray>("/kait_robot/output", 1);


	for(int i=0; i<MOTOR_NUMS; i++) {
		next_target[i] = 0.0;
		now_angle[i] = 0.0;
		old_angle[i] = 0.0;
		err_o[i] = 0.0;
		err_i[i] = 0.0;
		err_d[i] = 0.0;
		output[i] = 0;
		x[i] = 0.0;
	}
	time_o = ros::Time::now();

	ros::AsyncSpinner spinner(2);
	spinner.start();
	ros::Rate  loop(100);

	while(ros::ok())
	{
	{
		//std::lock_guard<std::mutex> lock(m);		

		// PID計算
		cal_pid();
		drive();

		publication();
	}

		loop.sleep();
	}
	for(int i=0; i<MOTOR_NUMS; i++)	motor[i]->move(0);
	spinner.stop();
}

void PidController::joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
	//std::lock_guard<std::mutex> lock(m);
	get_now_angle = true;
	for(int i=0; i<MOTOR_NUMS; i++) now_angle[i] = msg->position[i];
}

void PidController::next_target_callback(const std_msgs::Float64MultiArray& msg)
{
	//std::lock_guard<std::mutex> lock(m);
	get_next_target = true;
	for(int i=0; i<MOTOR_NUMS; i++) next_target[i] = msg.data[i];
}

void PidController::publication()
{
	std_msgs::Float64MultiArray buf;
	buf.data.resize(MOTOR_NUMS*5);
	for (int i = 0; i < MOTOR_NUMS; i++) {
		buf.data[i] = next_target[i];
		buf.data[i+8] = now_angle[i];
		buf.data[i+16] = err[i];
		buf.data[i+24] = x[i];
		buf.data[i+32] = output[i];
	}
	output_pub.publish(buf);
}

void PidController::cal_pid()
{
	ros::Time time_n = ros::Time::now();

	if(get_now_angle == true && get_next_target == true) {
		double dt = (time_n - time_o).toSec();
		double max[8]={},min[8]={};

		if(next_target[0] < 10) {
			//ROS_INFO("next target[0] : %.4f",next_target[0]);
			for(int i=0; i<MOTOR_NUMS; i++)
			{
				err[i] = next_target[i] - now_angle[i];
				if(fabs(err[i])<0.005) err[i] = 0.0; // とても重要。ハンチングがなくなる。
				err_i[i] += err[i] * dt;
				// PID
				//x[i] = Kp[i] * err[i] + Kd[i] / dt * (err[i] - err_o[i]) + Ki[i] * dt * err_i[i];
				// 不完全微分
				//err_d[i] = (1.0 - alpha) * err[i] + alpha * err_d[i]; 
				// PI-D　微分先行型PID制御 + 不完全微分
				x[i] = Kp[i] * err[i] - Kd[i] * (now_angle[i] - old_angle[i]) / dt + Ki[i] * err_i[i];
				if((double)x[i] != 0.0){
					if(max[i] < (double)x[i])
						max[i] = (double)x[i];
					else if(min[i] > (double)x[i])
						min[i] = (double)x[i];
				}
				//ROS_INFO("next_target[%d]:%.4f, now_angle[%d]:%.4f, old_angle[%d]:%.4f",i,(double)next_target[i],i,(double)now_angle[i],i,(double)old_angle[i]);
				
				//x[i] += Kp[i] * (err[i] - err_o[i]);// + Ki[i] * err[i] * dt;wd
				err_o[i] = err[i];
				
			}
			first_vel = true;
		}
		else {
			//ROS_INFO("next target[0] : %.4f",next_target[0]);
			if(first_vel == true) {
				for(int i=0;i<MOTOR_NUMS;i++) x[i] = 0.0;
				for(int i=0;i<MOTOR_NUMS;i+=2) lock_angle[i] = now_angle[i];
				first_vel = false;
				ROS_INFO("first_vel reset");
			}
			for(int i=0;i<MOTOR_NUMS;i++) {
				if(i%2==0) {
					err[i] = lock_angle[i] - now_angle[i];
					if(fabs(err[i])<0.005) err[i] = 0.0; // とても重要。ハンチングがなくなる。
					x[i] = 50000 * err[i];
				}
				else {
					if(fabs(next_target[i]) > 0.00001) x[i] = 0.01 * next_target[i] + 0.99 * x[i];
					else x[i] = 0.2 * next_target[i] + 0.8 * x[i];
				}
			}
		}
		
		
		//for(int i=0; i<MOTOR_NUMS; i++)
		//ROS_INFO("old_angle[%d]:%.4f,		now_angle[%d]:%.4f",i,(double)old_angle[i],i,(double)now_angle[i]);
		//ROS_INFO("------------------");
		for(int i=0; i<MOTOR_NUMS; i++)
		{
			old_angle[i] = now_angle[i];
		}
		
		// 出力変換
		for(int i=0; i<MOTOR_NUMS; i++)
		{
			if(i%2==1) {
				if(x[i] > 0.0) {
					output[i] = (int)(para[i].ap * (1.0 - exp(-x[i]/para[i].bp)));
				}
				else if(x[i] < 0.0) {
					output[i] = (int)(para[i].an * (1.0 - exp(-x[i]/para[i].bn)));
					//output[i] = (int)x[i];
				}
				else {
					output[i] = 0;
				}
			}
			else {
				output[i] = (int)x[i];
			}
			
			
			if(output[i] > 3500) output[i] = 3500;
			if(output[i] < -3500) output[i] = -3500;
		}
		//for(int i=0; i<MOTOR_NUMS; i++){
		//ROS_INFO("x[%d]:%.4f,	max[%d]:%.4f,	min[%d]:%.4f",i,(double)x[i],i,(double)max[i],i,(double)min[i]);
		//ROS_INFO("x[%d]:%.4f,	err[%d]:%.4f,	err_i[%d]:%.4f",i,(double)x[i],i,(double)err[i],i,(double)err_i[i]);
		//ROS_INFO("output[%d] : %.4f",i,(double)output[i]);
		//ROS_INFO("------------------");}
		/*//////////////////////////////////
		for(int i=0; i<MOTOR_NUMS; i++)
		ROS_INFO("output[%d]:%d,\tx[%d]:%.4f,\terr[%d]:%.4f",i,output[i],i,(double)x[i],i,(double)err[i]);
		ROS_INFO("------------------");
		//////////////////////////////////*/
		//ROS_INFO("%lf: [1]=%d, x=%lf, err=%lf", dt, output[1], x[1], err[1]);
	}
	time_o = time_n;
}

void PidController::drive()
{
	if(get_now_angle == true && get_next_target == true) {
		for(int i=0; i<MOTOR_NUMS; i++)	{
			if(output[i] > 10 || output[i] < -10) {
				if(i==1 || i==7) motor[i]->move(-output[i]); // wheel_flとwheel_blはモーター出力を逆転させる
				else motor[i]->move(output[i]);
			}
			else motor[i]->stop();
		}
	}
}
