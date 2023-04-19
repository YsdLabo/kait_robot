class WheelOdometry
{
private:
	double L1 = 0.4 + 0.1;
	double L2 = 0.4 * std::sqrt(2) + 0.1;
	double L3 = 0.2 * std::sqrt(2) + 0.05;
	double areaR_F = - 3.0 * M_PI / 8.0;
	double areaF_R = - M_PI / 8.0;
	double areaF_L = M_PI / 8.0;
	double areaL_F = 3.0 * M_PI / 8.0;
	double areaL_B = 5.0 * M_PI / 8.0;
	
	double cur_x;
	double cur_y;
	double cur_th;
	
	sensor_msgs::JointState wheel_state_last;
	bool first_run = true;
public:
	void run(sensor_msgs::JointState& wheel_state, double steer[4]);
	void publication();
};

WheelOdometry::WheelOdometry()
{
	cur_x = 0.0;
	cur_y = 0.0;
	cur_th = 0.0;
}

void WheelOdometry::update(sensor_msgs::JointState& wheel_state)
{
	wheel_state_last = wheel_state;
}

void WheelOdometry::update(sensor_msgs::JointState& wheel_state, double steer[4])
{
	double dv;
	double phi;
	
	if(!first_run) {
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
			if(diff_wheel[0] > 0) dth = (diff_wheel[1] - diff_whee[3]) / L2;
			// Backward Right
			else dth = (diff_wheel[3] - diff_wheel[1]) / L2;
			dv = (diff_wheel[0]+diff_wheel[1]+diff_wheel[2]+diff_wheel[3]) / 4.0;
			phi = M_PI / 4.0;
		}
		// FR & BL
		else if(steer[1] > areaR_F && steer[1] < areaF_R)
		{
			// Forward Right
			if(diff_wheel[0] > 0) dth = (diff_wheel[2] - diff_whee[0]) / L2;
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
		
		cur_th += dth;
		cur_x += dv * std::cos(theta + phi);
		cur_y += dv * std::sin(theta + phi);
		
		publication(wheel_state);
	}
	wheel_state_last = wheel_state;
	first_run = false;
}

void WheelOdometry::publication(sensor_msgs::JointState& wheel_state)
{
	geometry_msgs::Odometry odom;
	odom.header.stamp = wheel_state.header.stamp;
}

