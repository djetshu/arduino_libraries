#ifndef snake_robot_motor_h
#define snake_robot_motor_h
#define LIBRARY_VERSION	1.0.0

#include <PID_v1.h>
class Motor_Snake
{
  public:
  Motor_Snake(int, int, int,  // Motor driver pins
              const float, const float, const float, //Motor parameters
              const int, const int, // Encoder pins
              double*, double*, double*, double, double, double, float, float // PID parameters
              );

  Motor_Snake(int, int, int,  // Motor driver pins
              const float, const float, const float, //Motor parameters
              const int, const int, // Encoder pins
              double*, double*, double*, double, double, double, float, float, // PID parameters
              bool); // Interrupt flag

  //commonly used functions **************************************************************************
  void setup_motor_driver();
  void setup_encoder();
  void setup_pid();
  void setup_full_snake_motor();
  void setup_interruption();
  


  private:

  static void isrWrapper(); // Static member function as a wrapper
  static Motor_Snake* instance; // Static member to hold the current instance pointer
  void handleEncoder();


  //################ Variables for snake robot #############
  // ########## MOTOR DRIVER VARIABLES ##############
  int PWM ;
  int INA ;
  int INB ;
  // ########## MOTOR VARIABLES ##############
  const float PPR ;
  const float gearRatio ;
  const float decodeNumber ;
  double rotationalAngle = 0.0;
  // ########## ENCODER VARIABLES ##############
  const int encoderPinA ;  // Connect encoder channel A Motor 1 to digital pin 2
  const int encoderPinB ;  // Connect encoder channel B Motor 1 to digital pin 2
  volatile long encoderPosition = 0;
  // ########## PID VARIABLES ##############
  double *setpoint;
  double *input, *output;
  double Kp, Ki, Kd ;  // PID tuning parameters
  PID pid;
  //PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
  float current_error = 0.0;
  float limit_output_pid = 100;
  float sample_time_pid = 10;
  // ########## Interrupt VARIABLES ##############
  bool interrupt = false;
};
#endif