#include "motor_controller.h"

namespace driving_controller_ns
{

MotorController::MotorController()
{
  // Initialize Servo Motor
  ics_init(&ics_data);

  // Initialize Piezo Sonic Motor
  piezo[0].open(1);  // motor 2
  piezo[0].invert();
  piezo[1].open(3);  // motor 4
  piezo[2].open(5);  // motor 6
  piezo[3].open(7);  // motor 8
  piezo[3].invert();
  
  piezo[0].config(0x00, 0x2905);  // CW High Frequency 41.5Hz
  piezo[0].config(0x02, 0x2D07);  // CW Low Frequency 45.7Hz
  piezo[0].config(0x08, 0x2905);  // CCW High Frequency 41.5Hz
  piezo[0].config(0x0A, 0x2D07);  // CCW Low Frequency 45.7Hz
  piezo[0].config(0x12, 0);  // Kp
  piezo[0].config(0x14, 3900);  // Ki
  
  piezo[1].config(0x00, 0x2800);  // CW High Frequency 40.0Hz
  piezo[1].config(0x02, 0x2C00);  // CW Low Frequency 44.0Hz
  piezo[1].config(0x08, 0x2800);  // CCW High Frequency 40.0Hz
  piezo[1].config(0x0A, 0x2C00);  // CCW Low Frequency 44.0Hz
  piezo[1].config(0x12, 0);  // Kp
  piezo[1].config(0x14, 3900);  // Ki

  piezo[2].config(0x00, 0x2900);  // CW High Frequency 41.0Hz
  piezo[2].config(0x02, 0x2D07);  // CW Low Frequency 45.7Hz
  piezo[2].config(0x08, 0x2900);  // CCW High Frequency 41.0Hz
  piezo[2].config(0x0A, 0x2D07);  // CCW Low Frequency 45.7Hz
  piezo[2].config(0x12, 0);  // Kp
  piezo[2].config(0x14, 3100);  // Ki

  piezo[3].config(0x00, 0x2900);  // CW High Frequency 41.0Hz
  piezo[3].config(0x02, 0x2D07);  // CW Low Frequency 45.7Hz
  piezo[3].config(0x08, 0x2900);  // CCW High Frequency 41.0Hz
  piezo[3].config(0x0A, 0x2D07);  // CCW Low Frequency 45.7Hz
  piezo[3].config(0x12, 0);  // Kp
  piezo[3].config(0x14, 3100);  // Ki

  for(int i=0;i<4;i++) {
    piezo[i].config(0x04, 0x2400);  // CW Phase 36.0
    piezo[i].config(0x0C, 0x2400);  // CCW Phase 36.0
  }

  // Initialize Trapezoidal Controller
  for(int i=0;i<4;i++)
  {
    trape[i].SetAccMax(0.5);
    trape[i].SetDccMax(-0.5);
    trape[i].SetVelMax(1.0);
  }

  for(int i=0;i<4;i++) output[i] = 0;
  
  // Topic
  joint_states_sub = nh.subscribe("joint_states", 10, &MotorController::joint_states_callback, this);
  pub[0] = nh.advertise<std_msgs::Float64>("servo0", 1, this);
  pub[1] = nh.advertise<std_msgs::Float64>("servo1", 1, this);
  pub[2] = nh.advertise<std_msgs::Float64>("servo2", 1, this);
  pub[3] = nh.advertise<std_msgs::Float64>("servo3", 1, this);
  pub[4] = nh.advertise<std_msgs::Float64>("piezo0", 1, this);
  pub[5] = nh.advertise<std_msgs::Float64>("piezo1", 1, this);
  pub[6] = nh.advertise<std_msgs::Float64>("piezo2", 1, this);
  pub[7] = nh.advertise<std_msgs::Float64>("piezo3", 1, this);

  steer_last = steer_now = steer_next = 0;
}

MotorController::~MotorController()
{
  for(int i=0;i<4;i++) ics_free(&ics_data, i+1);
  ics_close(&ics_data);
  for(int i=0;i<4;i++) {
    piezo[i].stop();
  	piezo[i].close();
  }
}
  
void MotorController::drive_servo(int servo_id, int pulse, int speed)
{
  ics_set_speed(&ics_data, servo_id+1, std::abs(speed));
  if(pulse > 10462) pulse = 10462;
  if(pulse < 4537) pulse = 4537;
  ics_pos(&ics_data, servo_id+1, pulse);
}
  
void MotorController::drive_piezo(int piezo_id, int speed)
{
  int speed_m = speed;
  if(speed > 4000) speed_m = 4000;
  if(speed < -4000) speed_m = -4000;
  if(std::abs(speed) < 100) speed_m = 0;
  piezo[piezo_id].move(speed_m);
//  if(wheel_state.velocity.size() > 0)
//    printf("%d : %d : %lf\n", piezo_id, speed_m, wheel_state.velocity[piezo_id]);
}

bool MotorController::check_servo_stop(int servo_id)
{
  int cur_value = ics_get_position(&ics_data, servo_id+1);
//  printf("%d : %d    %d\n", servo_id+1, cur_pos, abs(cur_pos - steering_value[steer_next][servo_id]));
  if(std::abs(cur_value - steering_value[steer_next][servo_id]) < 60) return true;
  return false;
}

bool MotorController::check_all_servos_stop()
{
  int cnt = 0;
  for(int i=0;i<4;i++) {
    if(check_servo_stop(i)) cnt++;
  }
  if(cnt == 4) return true;
  return false;
}

void MotorController::set_piezo_goal_position(int piezo_id, int amount)
{
  double cur_pos = wheel_state.position[piezo_id];
  piezo_goal[piezo_id] = cur_pos + amount * M_PI * 0.25;
//  printf("id: %d  start: %lf - goal: %lf  = %lf\n", piezo_id, cur_pos, piezo_goal[piezo_id], piezo_goal[piezo_id]-cur_pos);
}

bool MotorController::check_piezo_stop(int piezo_id)
{
  double cur_pos = wheel_state.position[piezo_id];
  double err = cur_pos - piezo_goal[piezo_id];
//  printf("%lf : ", err);
  if(std::fabs(err) < 0.03) return true;
  return false;
}

bool MotorController::check_all_piezos_stop()
{
  static int sum = 0;
  int cnt = 0;
  if(wheel_state.header.seq > 0) {
    for(int i=0;i<4;i++) {
      if(std::fabs(wheel_state.velocity[i]) < 0.01) cnt++;
//      if(check_piezo_stop(i)) cnt++;
    }
  }
  if(cnt == 4) sum ++;
  else sum = 0;

  if(sum >= 5) {
  	return true;
  }
  return false;
}

void MotorController::joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
  wheel_state = *msg;
}

/*
void MotorController::steering(int next)
{
  int amount[4];

  steer_next = next;
  if(steer_next != steer_now) {
    steer_last = steer_now;
  }

  for(int i=0;i<4;i++) {
    amount[i] = steering_speed[steer_last][steer_next][i];
  }
  if(first_steering) {
    for(int i=0;i<4;i++)
      set_piezo_goal_position(i, amount[i]);
  	first_steering = false;
  }

  for(int i=0;i<4;i++) {
    //int amount = steering_speed[steer_last][steer_next][i];
    drive_servo(i, steering_value[steer_next][i], amount[i]*20);
    double err = piezo_goal[i] - wheel_state.position[i];
    //if(check_servo_stop(i)) drive_piezo(i, 0);
    if(check_piezo_stop(i)) drive_piezo(i, 0);
    //else drive_piezo(i, amount[i]*500+sign(amount[i])*2500);
    else drive_piezo(i, (int)(Kp[i]*err)+sign(err*100)*200);
//    printf("%d : ", (int)(Kp[i]*err)+sign(err*100)*200);
  }
//  printf("\n");
  
  steer_now = steer_next;
}
*/

void MotorController::idling()
{
  // Update Wheel Odometry
  odom.update(wheel_state);
  //ROS_INFO("idling odometry");
}

bool MotorController::steering(int steer_next_state)
{
  double pos_s_m[4];
  int pos_s[4];

  steer_next = steer_next_state;

  if(steer_next != steer_now) {
    steer_last = steer_now;
  }

  if(steering_flag == false)
  {
    for(int i=0;i<4;i++) {
      pos_p_m[i] = wheel_state.position[i]; // [rad]　車輪軸の現在角度
      pos_s_m[i] = servo_to_rad(i); // [rad]  操舵軸の現在角度
      double pos_s_d = (steering_value[steer_next][i] - 7500) / 4000.0 * 135.0 / 180.0 * M_PI; // [rad]  操舵軸の目標角度
      trape[i].Init(pos_s_d, pos_s_m[i]);    // 台形速度則の初期化
      pos_s_o[i] = pos_s_m[i];
    }
    steering_flag = true;  // 操舵中
  }
  else
  {
    double t_c = ros::Time::now().toSec();
    std_msgs::Float64 msg;
    
    // 台形速度則
    for(int i=0;i<4;i++) {
      // 操舵軸の中間目標角度
      pos_s_m[i] = trape[i].Next(t_c);    // [rad]
      // 車輪軸の中間目標角度
      if(i==0 || i==3) pos_p_m[i] -= pos_s_m[i] - pos_s_o[i];
      else pos_p_m[i] += pos_s_m[i] - pos_s_o[i];
      pos_s_o[i] = pos_s_m[i];
    }
    
    // モータ駆動
    for(int i=0;i<4;i++) {
      // 車輪軸駆動
      double pos_p = wheel_state.position[i]; // [rad]
      double err = pos_p_m[i] - pos_p;
      //drive_piezo(i, (int)(Kp[i]*err) + sign(err*100)*200);  // modify
      drive_piezo(i, (int)(Kp[i]*err));
      // 操舵軸駆動
      int pos_s_d = (int)(pos_s_m[i] * 180.0 / M_PI * 4000.0 / 135.0) + 7500;    // rad to digital
      drive_servo(i, pos_s_d, 100);
      // パブリッシュ
      msg.data = pos_p_m[i];
      pub[i].publish(msg);
      msg.data = pos_p;
      pub[i+4].publish(msg);
    }
    
    // update odom
    //odom.update(wheel_state);
    
    // 終了判定
    int cnt = 0;
    for(int i=0;i<4;i++) if(trape[i].Finished()) cnt++;
    if(cnt == 4) {
      for(int i=0;i<4;i++) drive_piezo(i, 0);    // 完全停止
      steering_flag = false;  // 操舵停止
      return true;
    }
  }
  // Update Wheel Odometry
  odom.update(wheel_state);
  //ROS_INFO("steering odometry");

  steer_now = steer_next;
  return false;
}

  
void MotorController::running(double speed_ms)
{
  double speed_d = speed_ms * mps_to_digit;
//  double rate[] = {0.92, 0.90, 1.0, 1.0};
  double rate[] = {1.0, 0, 0, 0};
  // Low-pass
  for(int i=0;i<4;i++) {
    if(speed_d == 0) output[i] = beta * output[i] + (1.0-beta) * speed_d * rate[i];  // when to stop
    else output[i] = alpha * output[i] + (1.0-alpha) * speed_d * rate[i];    // when to accelerate
  }
  ROS_INFO("speed_ms = %lf, speed_d = %lf, output[0] = %lf", speed_ms, speed_d, output[0]);

  double steer_dir[][4] = {
    { 1.0, 1.0,  1.0,  1.0}, // F, B
    { 1.0, 1.0,  1.0,  1.0}, // FL, FR
    { 1.0, 1.0,  1.0,  1.0}, // BL, BR
    {-1.0, 1.0, -1.0,  1.0}, // L, R
    {-1.0, 1.0,  1.0, -1.0}  // RotL, RotR
  };
  
  // F,B,FL,FR  0:+ 1:+ 2:+ 3:+
  for(int i=0;i<4;i++) {
    drive_piezo(i, steer_dir[steer_now][i]*output[i]);

/*    // F, B, FL, FR, BL, BR (state=0,1,2)
    if(steer_now < 3) {
      drive_piezo(i, (int)output[i]);
    }
    // L, R (state=3)
    else if (steer_now == 3) {
      if(i==0 || i==2) drive_piezo(i, -1*(int)output[i]);
      else drive_piezo(i, (int)output[i]);
    }
    // RotL, RotR (state=4)
    else {
      if(i==0 || i==3) drive_piezo(i, -1*(int)output[i]);
      else drive_piezo(i, (int)output[i]);
    }
*/
  }

  // Update Wheel Odometry
  for(int i=0;i<4;i++) steering_angle_now[i] = servo_to_rad(i);
  odom.update(wheel_state, steering_angle_now);
  //ROS_INFO("running odometry");
}

bool MotorController::go_to_home()
{
  if(!(wheel_state.header.seq > 0)) return false;

  if(steering(0)) {
    printf("I'm ready.\n");
    steer_last = 0;
    steer_now = 0;
    first_steering = true;
    return true;
  }
  return false;
}

void MotorController::steering_stop()
{
  first_steering = true;
}

double MotorController::servo_to_rad(int servo_id)
{
  int servo = ics_get_position(&ics_data, servo_id+1);
  if(servo < 0) return (double)servo;
  return (servo - 7500) / 4000.0 * 135.0 / 180.0 * M_PI;
}

}

