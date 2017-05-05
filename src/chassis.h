#ifndef CHASSIS_H
#define CHASSIS_H

#include <Servo.h>
#include <Encoder.h>

class Chassis{
public:
  Chassis();
  virtual ~Chassis();

  void initalize;
  int choseTask;
private:
  //Motor Controller for both Left and Right
  int MotorControllerPin;
  //Steer Controller, The Driving System
  int SteerControllerPin;

  int LeftEncoderPin;
  int RightEncoderPin;

  Encoder Motor_Encoder;
  Servo Motor_Controller;
  Servo Steer_Controller;
}
#endif
