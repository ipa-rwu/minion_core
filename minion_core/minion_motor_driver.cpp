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


bool MinionMotorDriver::moveMotor(Adafruit_DCMotor *motor, int16_t speed_percent)
{
  int16_t speed = constrain(abs(speed_percent), 0, 100);
  speed = speed * MINI_MAX_VELOCITY_MOTOR/100 ;
  Serial.print("speed: ");
  Serial.println(speed);
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


bool MinionMotorDriver::writeVelocity(int16_t left_value, int16_t right_value)
{
  Serial.print("left_value: ");
  Serial.println(left_value);
  Serial.print("right_value: ");
  Serial.println(right_value);

  moveMotor(_left_motor_pointer, -left_value);
  moveMotor(_right_motor_pointer, right_value);

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

  Serial.print("lin_vel: ");
  Serial.println(lin_vel);
  
  //  Calculate wheel speeds in m/s without encoder
  float left_speed = lin_vel - (ang_vel * wheel_separation / 2);
  float right_speed = lin_vel + (ang_vel * wheel_separation / 2);

  if (lin_vel == 0 && ang_vel != 0)
  {
    float left_speed = 0.2 - (ang_vel * wheel_separation / 2);
    float right_speed = 0.2 + (ang_vel * wheel_separation / 2); 
  }

  Serial.print("left_speed: ");
  Serial.println(left_speed);

  // Serial.print("_adafruit_limit_max_velocity: ");
  // Serial.println(_adafruit_limit_max_velocity);
  

  wheel_velocity_cmd[LEFT]   = 100 * left_speed/_adafruit_limit_max_velocity; 
  wheel_velocity_cmd[RIGHT]  = 100 * right_speed/_adafruit_limit_max_velocity;
  
  Serial.print("wheel_velocity_cmd LEFT: ");
  Serial.println(wheel_velocity_cmd[LEFT]);

  Serial.print("wheel_velocity_cmd RIGHT: ");
  Serial.println(wheel_velocity_cmd[RIGHT]);
  comm_result = writeVelocity((int16_t)wheel_velocity_cmd[LEFT], (int16_t)wheel_velocity_cmd[RIGHT]);
  if (comm_result == false)
    return false;

  return true;
}
