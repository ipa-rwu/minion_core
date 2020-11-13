#ifndef MINION_MINI_H_
#define MINION_MINI_H_

//https://emanual.robotis.com/docs/en/platform/turtlebot3/specifications/#hardware-specifications
#define NAME                             "mini"

#define WHEEL_RADIUS                     0.0325           // meter
#define WHEEL_SEPARATION                 0.130           // meter 
#define TURNING_RADIUS                   0.075           // meter 
#define ROBOT_RADIUS                     0.209           // meter (BURGER : 0.105, WAFFLE : 0.220)
#define ENCODER_MIN                      -2147483648     // raw
#define ENCODER_MAX                      2147483648      // raw

#define MAX_LINEAR_VELOCITY              (WHEEL_RADIUS * 2 * 3.14159265359 * 61 / 60) // m/s  (BURGER : 61[rpm], WAFFLE : 77[rpm])
#define MAX_ANGULAR_VELOCITY             (MAX_LINEAR_VELOCITY / TURNING_RADIUS)       // rad/s

#define MIN_LINEAR_VELOCITY              -MAX_LINEAR_VELOCITY  
#define MIN_ANGULAR_VELOCITY             -MAX_ANGULAR_VELOCITY 

#endif  //MINION_MINI_H_
