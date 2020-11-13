#include "minion_motor_driver.h"

MinionMotorDriver::MinionMotorDriver()
: left_wheel_id_(ADAFRUIT_LEFT_ID),
  right_wheel_id_(ADAFRUIT_RIGHT_ID)
{
  torque_ = false;
  adafruit_limit_max_velocity_ = MINI_LIMIT_MAX_VELOCITY;
  AFMS_ = Adafruit_MotorShield(); 
  left_motor_pointer_ = AFMS_.getMotor(left_wheel_id_);
  right_motor_pointer_ = AFMS_.getMotor(right_wheel_id_);


}

MinionMotorDriver::~MinionMotorDriver()
{
}

bool MinionMotorDriver::init(String minion)
{
  Serial.begin(57600);
  AFMS_.begin();

  if (minion == "mini")
    adafruit_limit_max_velocity_ = MINI_LIMIT_MAX_VELOCITY;

  Serial.println("Success to init Motor Driver");
  return true;
}



bool MinionMotorDriver::writeVelocity(int64_t left_value, int64_t right_value)
{

  if (left_value >= 0)
  {
    left_motor_pointer_ -> setSpeed(left_value);
    left_motor_pointer_ ->run(FORWARD);
  }

  if (left_value < 0)
  {
    left_motor_pointer_ -> setSpeed(left_value);
    left_motor_pointer_ ->run(BACKWARD);
  }

  if (right_value >= 0)
  {
    right_motor_pointer_ -> setSpeed(right_value);
    right_motor_pointer_ ->run(FORWARD);
  }

  if (right_value < 0)
  {
    right_motor_pointer_ -> setSpeed(right_value);
    right_motor_pointer_ ->run(BACKWARD);
  }
  
  
  return true;
}

bool MinionMotorDriver::controlMotor(const float wheel_radius, const float wheel_separation, float* value)
{
  bool comm_result = false;
  
  float wheel_velocity_cmd[2];

  float lin_vel = value[LEFT];
  float ang_vel = value[RIGHT];

  wheel_velocity_cmd[LEFT]   = lin_vel - (ang_vel * wheel_separation / 2);
  wheel_velocity_cmd[RIGHT]  = lin_vel + (ang_vel * wheel_separation / 2);

  wheel_velocity_cmd[LEFT]  = constrain(wheel_velocity_cmd[LEFT]  * VELOCITY_CONSTANT_VALUE / wheel_radius, -adafruit_limit_max_velocity_, adafruit_limit_max_velocity_);
  wheel_velocity_cmd[RIGHT] = constrain(wheel_velocity_cmd[RIGHT] * VELOCITY_CONSTANT_VALUE / wheel_radius, -adafruit_limit_max_velocity_, adafruit_limit_max_velocity_);

  comm_result = writeVelocity((int64_t)wheel_velocity_cmd[LEFT], (int64_t)wheel_velocity_cmd[RIGHT]);
  if (comm_result == false)
    return false;

  return true;
}
