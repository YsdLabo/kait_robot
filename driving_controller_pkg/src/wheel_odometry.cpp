void WheelOdometry::run(wheel, steer)
{
	// Wheel Odometry for 4WS
	int steering_dir;    // 0:F&B, 1:FL&BR, 2:FR&BL, 3:L&R, 4:Rotation

	// F&B
	if((steer[0] > -22.5= && steer[0] <= 22.5)) steering_dir = 0;
	// FL&BR
	else if((steer[0] > -67.5 && steer[0] < -22.5)) steering_dir = 1;
	// FR&BL
	else if((steer[0] > 22.5 && steer[0] < 67.5) && (steer[1] < 0)) steering_dir = 2;
	// L&R
	else if((steer[0] > 67.5 && steer[0] < 112.5)) steering_dir = 3;
	// Rotation
	else if((steer[0] > 22.5 && steer[0] < 67.5) && (steer[1] > 0)) steering_dir = 4;
		
	// 操舵方向に合わせて，オドメトリ
	if(steering_dir == 0)
	{
	}
	else if(steering_dir == 1)
	{
	}
	else if(steering_dir == 2)
	{
	}
	else if(steering_dir == 3)
	{
	}
	else if(steering_dir == 4)
	{
	}
}

void WheelOdometry::publication()
{
}

