#define USE_USBCON

#include "minion_core_config.h"


void setup() {
  Serial.begin(57600);
  nh.initNode();
  nh.getHardware()->setBaud(115200);

  nh.subscribe(cmd_vel_sub);

  motor_driver.init(NAME);
    
}

void loop() {

  uint32_t t = millis();
  updateTime();

  if ((t-tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_FREQUENCY))
  {
    updateGoalVelocity();
    if ((t-tTime[6]) > CONTROL_MOTOR_TIMEOUT) 
    {
      motor_driver.controlMotor(WHEEL_RADIUS, WHEEL_SEPARATION, zero_velocity);
    } 
    else {
      motor_driver.controlMotor(WHEEL_RADIUS, WHEEL_SEPARATION, goal_velocity);
    }
    tTime[0] = t;

  }
  
  // Call all the callbacks waiting to be called at that point in time
  nh.spinOnce();
}



/*******************************************************************************
* Update the base time for interpolation
*******************************************************************************/
void updateTime()
{
  current_offset = millis();
  current_time = nh.now();
}

/*******************************************************************************
* ros::Time::now() implementation
*******************************************************************************/
ros::Time rosNow()
{
  return nh.now();
}

/*******************************************************************************
* Time Interpolation function (deprecated)
*******************************************************************************/
ros::Time addMicros(ros::Time & t, uint32_t _micros)
{
  uint32_t sec, nsec;

  sec  = _micros / 1000 + t.sec;
  nsec = _micros % 1000000000 + t.nsec;

  return ros::Time(sec, nsec);
}

/*******************************************************************************
* Callback function for cmd_vel msg
*******************************************************************************/
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
  goal_velocity_from_cmd[LINEAR]  = cmd_vel_msg.linear.x;
  goal_velocity_from_cmd[ANGULAR] = cmd_vel_msg.angular.z;

  goal_velocity_from_cmd[LINEAR]  = constrain(goal_velocity_from_cmd[LINEAR],  MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
  goal_velocity_from_cmd[ANGULAR] = constrain(goal_velocity_from_cmd[ANGULAR], MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
  tTime[6] = millis();
}


void updateGoalVelocity(void)
{
  goal_velocity[LINEAR]  = goal_velocity_from_cmd[LINEAR] ;
  goal_velocity[ANGULAR] = goal_velocity_from_cmd[ANGULAR];
}
