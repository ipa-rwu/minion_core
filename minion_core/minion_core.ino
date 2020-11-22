#define USE_USBCON

#include "minion_core_config.h"


void setup() {
  Serial.begin(57600);

  // Initialize ROS node handle, advertise and subscribe the topics
  nh.initNode();
  nh.getHardware()->setBaud(115200);

  nh.subscribe(cmd_vel_sub);

  motor_driver.init(NAME);
  // tf_broadcaster.init(nh);

  goal_velocity[LINEAR]  = 0;
  goal_velocity[ANGULAR] = 0;
  motor_driver.controlMotor(WHEEL_SEPARATION, goal_velocity);  

  memset(tTime,0,sizeof(tTime));
  //tTime[6] = millis();
  //tTime[0] = millis();
    
}

void loop() {

  uint32_t t = millis();
  updateTime();
  updateTFPrefix(nh.connected());

  // updateGoalVelocity();
  // motor_driver.controlMotor(WHEEL_SEPARATION, goal_velocity);

  // if ((t-tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_FREQUENCY))
  // {
    updateGoalVelocity();
    // tTime[6] the time when get cmd_vel
    if ((t-tTime[6]) > CONTROL_MOTOR_TIMEOUT) 
    {
      motor_driver.controlMotor(WHEEL_SEPARATION, zero_velocity);
    } 
    else 
    {
      motor_driver.controlMotor(WHEEL_SEPARATION, goal_velocity);
    }
    // tTime[0] the time when update motor_driver
    // tTime[0] = t;

  // }

  if ((t-tTime[3]) >= (1000 / IMU_PUBLISH_FREQUENCY))
  {
    // publishImuMsg();
    // publishMagMsg();
    // tTime[0] the time when update imu
    tTime[3] = t;
  }

  
  // Call all the callbacks waiting to be called at that point in time
  nh.spinOnce();
  // delay(10);

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


  // goal_velocity_from_cmd[LINEAR]  = constrain(goal_velocity_from_cmd[LINEAR],  MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
  // goal_velocity_from_cmd[ANGULAR] = constrain(goal_velocity_from_cmd[ANGULAR], MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
  tTime[6] = millis();
}


void updateGoalVelocity(void)
{
  goal_velocity[LINEAR]  = goal_velocity_from_cmd[LINEAR] ;
  goal_velocity[ANGULAR] = goal_velocity_from_cmd[ANGULAR];
  // motor_driver.controlMotor(WHEEL_SEPARATION, goal_velocity);  

}


/*******************************************************************************
* Update TF Prefix
*******************************************************************************/
void updateTFPrefix(bool isConnected)
{
  static bool isChecked = false;
  char log_msg[50];

  if (isConnected)
  {
    if (isChecked == false)
    {
      nh.getParam("~tf_prefix", &get_tf_prefix);

      if (!strcmp(get_tf_prefix, ""))
      {
        // sprintf(odom_header_frame_id, "odom");
        // sprintf(odom_child_frame_id, "base_footprint");  

        sprintf(imu_frame_id, "imu_link");
        sprintf(mag_frame_id, "mag_link");
        sprintf(joint_state_header_frame_id, "base_link");
      }
      else
      {
        // strcpy(odom_header_frame_id, get_tf_prefix);
        // strcpy(odom_child_frame_id, get_tf_prefix);

        strcpy(imu_frame_id, get_tf_prefix);
        strcpy(mag_frame_id, get_tf_prefix);
        strcpy(joint_state_header_frame_id, get_tf_prefix);

        // strcat(odom_header_frame_id, "/odom");
        // strcat(odom_child_frame_id, "/base_footprint");

        strcat(imu_frame_id, "/imu_link");
        strcat(mag_frame_id, "/mag_link");
        strcat(joint_state_header_frame_id, "/base_link");
      }

      // sprintf(log_msg, "Setup TF on Odometry [%s]", odom_header_frame_id);
      // nh.loginfo(log_msg); 

      sprintf(log_msg, "Setup TF on IMU [%s]", imu_frame_id);
      nh.loginfo(log_msg); 

      sprintf(log_msg, "Setup TF on MagneticField [%s]", mag_frame_id);
      nh.loginfo(log_msg); 

      sprintf(log_msg, "Setup TF on JointState [%s]", joint_state_header_frame_id);
      nh.loginfo(log_msg); 

      isChecked = true;
    }
  }
  else
  {
    isChecked = false;
  }
}
