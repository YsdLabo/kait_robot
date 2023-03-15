#include "motor_controller.h"


void MotorController::drive_servo(int motor_id, int angle, int speed)
{
  ics_set_speed(&ics_data, motor_id, std::abs(speed) * 20);
  int pulse = angle_to_pulse(angle);
  ics_pos(&ics_data, motor_id, pulse);
}
  
void MotorController::drive_piezo(int motor_id, int speed)
{
  int speed_m = speed;
  if(speed > 4000) speed_m = 4000;
  if(speed < -4000) speed_m = -4000;
  piezo[motor_id].move(speed_m);
}
  
int MotorController::angle_to_pulse(int angle)
{
  int pulse = (int)(angle * 4000.0 / 135.0) + 7500;
  if(pulse < 3500) pulse = 3500;
  if(pulse > 11500) pulse = 11500;
  return pulse;
}

void MotorController::joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
  
}

MotorController::SteeringController() {
  ics_init(&ics_data);
  for(int i=0;i<4;i++) piezo[i].open(i);
  piezo[0].invert();
  piezo[3].invert();
  nh = getNodeHandle();
  joint_states_sub = nh.subscribe("/kait_robot/joint_states", 10, &MotorController::joint_states_callback);
}
  
MotorController::~SteeringController() {
  ics_close(&ics_data);
  delete [] piezo;
  for(int i=0;i<4;i++) piezo[i].close();
}

void MotorController::steering(int steer_next)
{
  int amount[4];
  for(int i=0;i<4;i++) {
    amount[i] = (steering_angle[steer_next][i] - steering_angle[steer_now][i]) / 45;
    drive_servo(i, steering_angle[steer_next][i], amount[i]);
    if(check_servo
    drive_piezo(i, amount[i]);
  }
  
  steer_now = steer_next;
}
  
void MotorController::running(double speed_ms)
{
  double speed = speed_ms * mps_to_digit;
  // Low-pass
  for(int i=0;i<4;i++) {
    if(speed == 0) output[i] = beta * output[i] + (1.0-beta) * speed;  // when to stop
    else output[i] = alpha * output[i] + (1.0-alpha) * speed;    // when to accelerate
  }
  
  // forward  0:- 1:+ 2:+ 3:-
  for(int i=0;i<4;i++) drive_piezo(i, output[i]);
}

void MotorController::steering_ready()
{
}
