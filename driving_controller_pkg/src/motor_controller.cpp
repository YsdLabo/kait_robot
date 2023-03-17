#include "motor_controller.h"


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
  piezo[motor_id].move(speed_m);
}

bool MotorController::check_servo_stop(int id)
{
  int pos = ics_get_position(&ics_data, id+1);
  if(abs(pos - steering_angle[steer_next][id]) < 4) return true;
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
  int cnt = 0;
  for(int i=1;i<8;i+=2) {
    if(joint_state.velocity[i] < 0.01) cnt++;
  }
  if(cnt == 4) return true;
  return false;
}

void MotorController::joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
  joint_state = *msg;
}

MotorController::MotorController()
{
  ics_init(&ics_data);
  for(int i=0;i<4;i++) piezo[i].open(2*i+1);  // open 2, 4, 6, 8
  piezo[0].invert();
  piezo[3].invert();
//  nh = getNodeHandle();
  joint_states_sub = nh.subscribe("/kait_robot/joint_states", 10, &MotorController::joint_states_callback, this);
  steer_last = steer_now = steer_next = 0;
}
  
MotorController::~MotorController()
{
  ics_close(&ics_data);
  for(int i=0;i<4;i++) piezo[i].close();
}

void MotorController::steering(int next)
{
  steer_next = next;
  if(steer_next != steer_now) {
    steer_last = steer_now;
  }
  for(int i=0;i<4;i++) {
    int amount = steering_speed[steer_last][steer_next][i];
    drive_servo(i, steering_angle[steer_next][i], amount*20);
    if(check_servo_stop(i)) drive_piezo(i, 0);
    else drive_piezo(i, amount*500);
  }
  
  steer_now = steer_next;
}
  
void MotorController::running(double speed_ms)
{
  double speed_d = speed_ms * mps_to_digit;
  // Low-pass
  for(int i=0;i<4;i++) {
    if(speed_d == 0) output[i] = beta * output[i] + (1.0-beta) * speed_d;  // when to stop
    else output[i] = alpha * output[i] + (1.0-alpha) * speed_d;    // when to accelerate
    if(output[i] < 10) output[i] = 0;
  }
  
  // F,B,FL,FR  0:+ 1:+ 2:+ 3:+
  for(int i=0;i<4;i++) {
    // F, B, FL, FR, BL, BR
    if(steer_next < 4) {
      drive_piezo(i, (int)output[i]);
    }
    // L, R, RotL, RotR
    else {
      // L,Rot.L  0:- 1:+ 2:- 3:+
      if(speed_ms > 0) {
        if(i%2 == 0) drive_piezo(i, -1*(int)output[i]);
        else drive_piezo(i, (int)output[i]);
      }
      // R,Rot.R  0:+ 1:- 2:+ 3:-
      else {
        if(i%2 == 0) drive_piezo(i, (int)output[i]);
        else drive_piezo(i, -1*(int)output[i]);
      }
    }
  }     
}

