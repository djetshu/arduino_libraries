#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <snake_robot_motor.h>
#include <PID_v1.h>

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
Motor_Snake::Motor_Snake(int pinPWM, int pinINA, int pinINB,  
                          const float PPR_motor, const float gearRatio_motor, const float decodeNumber_motor, 
                          const int encoder_PinA, const int encoder_PinB, 
                          double* InputPID, double* OutputPID, double* SetpointPID, 
                          double KpPID, double KiPID, double KdPID, float limitOutputPID, float sampleTimePID)
    : pid(InputPID, OutputPID, SetpointPID, KpPID, KiPID, KdPID, DIRECT),  // Initialize PID object here
      PWM(pinPWM), INA(pinINA), INB(pinINB),
      PPR(PPR_motor), gearRatio(gearRatio_motor), decodeNumber(decodeNumber_motor),
      encoderPinA(encoder_PinA), encoderPinB(encoder_PinB),
      setpoint(SetpointPID), input(InputPID), output(OutputPID),
      Kp(KpPID), Ki(KiPID), Kd(KdPID),
      current_error(0.0), limit_output_pid(limitOutputPID), sample_time_pid(sampleTimePID),
      interrupt(false) {
    // Constructor body if needed
}

Motor_Snake::Motor_Snake(int pinPWM, int pinINA, int pinINB,  
                          const float PPR_motor, const float gearRatio_motor, const float decodeNumber_motor, 
                          const int encoder_PinA, const int encoder_PinB, 
                          double* InputPID, double* OutputPID, double* SetpointPID, 
                          double KpPID, double KiPID, double KdPID, float limitOutputPID, float sampleTimePID,
                          bool interruptFlag)
    : pid(InputPID, OutputPID, SetpointPID, KpPID, KiPID, KdPID, DIRECT),  // Initialize PID object here
      PWM(pinPWM), INA(pinINA), INB(pinINB),
      PPR(PPR_motor), gearRatio(gearRatio_motor), decodeNumber(decodeNumber_motor),
      encoderPinA(encoder_PinA), encoderPinB(encoder_PinB),
      setpoint(SetpointPID), input(InputPID), output(OutputPID),
      Kp(KpPID), Ki(KiPID), Kd(KdPID),
      current_error(0.0), limit_output_pid(limitOutputPID), sample_time_pid(sampleTimePID),
      interrupt(interruptFlag) {
    // Constructor body if needed
}

void Motor_Snake::setup_motor_driver(){
  pinMode(PWM, OUTPUT);
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
}

void Motor_Snake::setup_encoder(){
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
}

Motor_Snake* Motor_Snake::instance = nullptr;

void Motor_Snake::isrWrapper() {
    // Call the handleEncoder() member function of the current instance
    if (instance) {
        instance->handleEncoder();
    }
}

void Motor_Snake::setup_interruption() {
    // Set the static member instance to the current instance
    instance = this;

    // Attach the interrupt using isrWrapper as the ISR
    attachInterrupt(digitalPinToInterrupt(encoderPinA), isrWrapper, CHANGE);
}

void Motor_Snake::setup_pid(){
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-limit_output_pid, limit_output_pid);  // Adjust output limits based on your motor driver
  pid.SetSampleTime(sample_time_pid);  // Set PID sample time in milliseconds
}

void Motor_Snake::setup_full_Snake_motor(){
  Motor_Snake::setup_encoder();
  Motor_Snake::setup_motor_driver();
  Motor_Snake::setup_pid();
  if (interrupt == true) Motor_Snake::setup_interruption();

}

void Motor_Snake::handleEncoder() {
  // Read the current state of the two channels
  int stateA = digitalRead(encoderPinA);
  int stateB = digitalRead(encoderPinB);

  // Update the encoder position based on the quadrature encoding
  if (stateA == stateB) {
    // Clockwise rotation
    encoderPosition++;
  } else {
    // Counterclockwise rotation
    encoderPosition--;
  }
}