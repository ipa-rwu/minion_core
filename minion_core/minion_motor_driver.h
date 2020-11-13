#ifndef MINION_MOTOR_DRIVER_H_
#define MINION_MOTOR_DRIVER_H_

#include <Adafruit_MotorShield.h>

// Limit values (XM430-W210-T and XM430-W350-T)
#define MINI_MAX_VELOCITY            0.5 
#define MINI_MAX_VELOCITY_MOTOR            255  


// Data Byte Length
#define LEN_X_TORQUE_ENABLE             1
#define LEN_X_GOAL_VELOCITY             4
#define LEN_X_GOAL_POSITION             4
#define LEN_X_REALTIME_TICK             2
#define LEN_X_PRESENT_VELOCITY          4
#define LEN_X_PRESENT_POSITION          4


#define ADAFRUIT_LEFT_ID                     4       // ID of left motor
#define ADAFRUIT_RIGHT_ID                    3       // ID of right motor


#define TORQUE_ENABLE                   1       // Value for enabling the torque
#define TORQUE_DISABLE                  0       // Value for disabling the torque

#define LEFT                            0
#define RIGHT                           1

#define LINEAR                           0
#define ANGULAR                          1           

#define VELOCITY_CONSTANT_VALUE         41.69988758  // V = r * w = r     *        (RPM             * 0.10472)
                                                     //           = r     * (0.229 * Goal_Velocity) * 0.10472
                                                     //
                                                     // Goal_Velocity = V / r * 41.69988757710309
class MinionMotorDriver
{
 public:
  MinionMotorDriver();
  ~MinionMotorDriver();
  bool init(String minion);
  void close(void);
  bool setTorque(bool onoff);
  bool getTorque();
  bool readEncoder(int32_t &left_value, int32_t &right_value);

  bool writeVelocity(int64_t left_value, int64_t right_value);

  bool controlMotor(const float wheel_separation, float* value);

  bool moveMotor(Adafruit_DCMotor *motor, int64_t speed_percent);


 private:
  uint8_t left_wheel_id_;
  uint8_t right_wheel_id_;
  Adafruit_DCMotor *_left_motor_pointer;
  Adafruit_DCMotor *_right_motor_pointer;

  bool torque_;
  Adafruit_MotorShield _AFMS;
  uint16_t _adafruit_limit_max_velocity;
};

#endif // MINION_MOTOR_DRIVER_H_
