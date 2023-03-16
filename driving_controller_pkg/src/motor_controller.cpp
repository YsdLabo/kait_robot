#include "motor_controller.h"


void MotorController::drive_servo(int motor_id, int pulse, int speed)
{
  ics_set_speed(&ics_data, motor_id, std::abs(speed) * 20);
  ics_pos(&ics_data, motor_id, pulse);
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
  int pos = ics_get_position(&ics_data, id);
  if(abs(pos - string_angle[steer_now][i]) < 4) return true;
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

void MotorController::joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
  
}

MotorController::SteeringController()
{
  ics_init(&ics_data);
  for(int i=0;i<4;i++) piezo[i].open(i);
  piezo[0].invert();
  piezo[3].invert();
  nh = getNodeHandle();
  joint_states_sub = nh.subscribe("/kait_robot/joint_states", 10, &MotorController::joint_states_callback);
  steer_last = steer_now = static_cast<E_STEERING>(E_STEERING::DIRECTION_STOP);
}
  
MotorController::~SteeringController()
{
  ics_close(&ics_data);
  delete [] piezo;
  for(int i=0;i<4;i++) piezo[i].close();
}

void MotorController::steering(int steer_next)
{
  if(steer_next != steer_now) {
    steer_last = steer_now;
  }
  for(int i=0;i<4;i++) {
    int amount = steering_speed[steer_last][steer_next][i];
    drive_servo(i, steering_angle[steer_next][i], amount);
    if(check_servo_stop(i)) drive_piezo(i, 0);
    else drive_piezo(i, amount*500);
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

