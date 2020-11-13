#include "minion_motor_driver.h"

MinionMotorDriver::MinionMotorDriver()
: left_wheel_id_(ADAFRUIT_LEFT_ID),
  right_wheel_id_(ADAFRUIT_RIGHT_ID)
{
  torque_ = false;
  // adafruit_limit_max_velocity_ = MINI_LIMIT_MAX_VELOCITY;
  _AFMS = Adafruit_MotorShield(); 
  _left_motor_pointer = _AFMS.getMotor(left_wheel_id_);
  _right_motor_pointer = _AFMS.getMotor(right_wheel_id_);


}

MinionMotorDriver::~MinionMotorDriver()
{
}

bool MinionMotorDriver::init(String minion)
{
  Serial.begin(57600);
  _AFMS.begin();

  if (minion == "mini")
    _adafruit_limit_max_velocity = MINI_MAX_VELOCITY;

  Serial.println("Success to init Motor Driver");
  return true;
}


bool MinionMotorDriver::moveMotor(Adafruit_DCMotor *motor, int64_t speed_percent)
{
  int64_t speed = constrain(abs(speed_percent), 0, 100);
  speed = speed /100 * MINI_MAX_VELOCITY_MOTOR;
  // Positive speeds move wheels forward,
  // negative speeds move wheels backward
  if (speed_percent < 0)
  {
    motor->setSpeed(speed);
    motor->run(BACKWARD);
    return true;
  }
  if (speed_percent > 0)
  {
    motor->setSpeed(speed);
    motor->run(FORWARD);
    return true;
  }
  if (speed_percent == 0)
  {
    motor->setSpeed(speed);
    motor->run(RELEASE);
    return true;
  }
  
}


bool MinionMotorDriver::writeVelocity(int64_t left_value, int64_t right_value)
{
  moveMotor(_left_motor_pointer, left_value);
  moveMotor(_right_motor_pointer, left_value);

  return true;
}

// without encoder
// wheel_separation = _wheel_base
bool MinionMotorDriver::controlMotor(const float wheel_separation, float* goal_velocity)
{
  bool comm_result = false;
  
  float wheel_velocity_cmd[2];

  float lin_vel = goal_velocity[LINEAR];
  float ang_vel = goal_velocity[ANGULAR];

  //  Calculate wheel speeds in m/s without encoder
  float left_speed = lin_vel - (ang_vel * wheel_separation / 2);
  float right_speed = lin_vel + (ang_vel * wheel_separation / 2);

  wheel_velocity_cmd[LEFT]   = 100 * left_speed/_adafruit_limit_max_velocity; 
  wheel_velocity_cmd[RIGHT]  = 100 * right_speed/_adafruit_limit_max_velocity;

  comm_result = writeVelocity((int64_t)wheel_velocity_cmd[LEFT], (int64_t)wheel_velocity_cmd[RIGHT]);
  if (comm_result == false)
    return false;

  return true;
}
