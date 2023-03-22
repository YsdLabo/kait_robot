#include "motor_controller.h"

namespace driving_controller_ns
{

MotorController::MotorController()
{
  ics_init(&ics_data);
  for(int i=0;i<4;i++) piezo[i].open(2*i+1);  // open 2, 4, 6, 8
  piezo[0].invert();
  piezo[3].invert();
  for(int i=0;i<4;i++) output[i] = 0;
  //nh = getNodeHandle();
  joint_states_sub = nh.subscribe("/kait_robot/joint_states", 10, &MotorController::joint_states_callback, this);
  steer_last = steer_now = steer_next = 0;
  //joint_state.header.seq = 0;
  //go_to_home();
}

MotorController::~MotorController()
{
  ics_close(&ics_data);
  for(int i=0;i<4;i++) {
    piezo[i].stop();
  	piezo[i].close();
  }
}
  
void MotorController::drive_servo(int motor_id, int pulse, int speed)
{
  ics_set_speed(&ics_data, motor_id+1, std::abs(speed));
  ics_pos(&ics_data, motor_id+1, pulse);
}
  
void MotorController::drive_piezo(int motor_id, int speed)
{
  int speed_m = speed;
  if(speed > 4000) speed_m = 4000;
  if(speed < -4000) speed_m = -4000;
  if(std::abs(speed) < 100) speed_m = 0;
  piezo[motor_id].move(speed_m);
}

bool MotorController::check_servo_stop(int motor_id)
{
  int pos = ics_get_position(&ics_data, motor_id+1);
//  printf("%d : %d    %d\n", motor_id+1, pos, abs(pos - steering_angle[steer_next][motor_id]));
  if(std::abs(pos - steering_angle[steer_next][motor_id]) < 20) return true;
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

bool MotorController::check_all_piezos_stop()
{
  static int sum = 0;
  int cnt = 0;
  if(joint_state.header.seq > 0) {
  //printf("check velocity  ");
    for(int i=1;i<8;i+=2) {
      //printf("%lf :", joint_state.velocity[i]*M_PI*0.05);
      if(std::fabs(joint_state.velocity[i]) < 0.01) cnt++;
    }
  //printf(" %d\n", cnt);
  }
  if(cnt == 4) {
  	sum ++;
  }
  else sum = 0;

  if(sum >= 5) {
  	sum = 0;
  	return true;
  }
  return false;
}

void MotorController::joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
  joint_state = *msg;
}

void MotorController::steering(int next)
{
  //printf("Steering : %d\n", next);
  steer_next = next;
  if(steer_next != steer_now) {
    steer_last = steer_now;
  }
  for(int i=0;i<4;i++) {
    int amount = steering_speed[steer_last][steer_next][i];
    drive_servo(i, steering_angle[steer_next][i], amount*20);
    if(check_servo_stop(i)) drive_piezo(i, 0);
    else drive_piezo(i, amount*500+sign(amount)*3000);
  }
  
  steer_now = steer_next;
}
  
void MotorController::running(double speed_ms)
{
  double speed_d = speed_ms * mps_to_digit;
  double rate[] = {1.0, 0.76, 1.03, 1.0};
  // Low-pass
  for(int i=0;i<4;i++) {
    if(speed_d == 0) output[i] = beta * output[i] + (1.0-beta) * speed_d * rate[i];  // when to stop
    else output[i] = alpha * output[i] + (1.0-alpha) * speed_d * rate[i];    // when to accelerate
  }
  printf("desired velocity   %lf : ", speed_d);
  for(int i=0;i<4;i++) {
    printf("%d :", (int)output[i]);
  }
  printf("\n");

  // F,B,FL,FR  0:+ 1:+ 2:+ 3:+
  for(int i=0;i<4;i++) {
    // F, B, FL, FR, BL, BR (0,1,2)
    if(steer_next < 3) {
      drive_piezo(i, (int)output[i]);
    }
    // L, R, RotL, RotR (3,4)
    else {
      if(i%2 == 0) drive_piezo(i, -1*sign(speed_ms)*(int)output[i]);
      else drive_piezo(i, sign(speed_ms)*(int)output[i]);
    }
  }     
}

bool MotorController::go_to_home()
{
  for(int i=0;i<4;i++) {
    int pos = ics_get_position(&ics_data, i+1);
    int pulse = 7500 - pos;//0:-+, 1:--, 2:--, 3:-+
    if(i==0 || i==3) drive_piezo(i, sign(pulse)*(-3000));
    else drive_piezo(i, sign(pulse)*3000);
    drive_servo(i, 7500, 20);
  }
  steer_next = 0;

  while(!check_all_servos_stop());
  printf("servo stop\n");
  
  for(int i=0;i<4;i++) {
    drive_piezo(i, 0);
  }
  //while(!check_all_piezos_stop());
  printf("piezo stop\n");
  return true;
}

}

