#ifndef CHASSIS_H
#define CHASSIS_H

#include <Servo.h>
#include <Encoder.h>

class Chassis{
public:
  Chassis();
  virtual ~Chassis();

  void initalize;

  void drive(double speed, int distance, int detection);
  void turn(int angle, int distance, double speed, int detection);


  double getEncoderData;
  int choseTask;
private:
  //Motor Controller for both Left and Right
  int MotorControllerPin;
  //Steer Controller, The Driving System
  int SteerControllerPin;

  int LeftEncoderPin;
  int RightEncoderPin;

  /*
  * Steering Ranges
  * Note that all of these values are relative to the
  * Servos positions. In our case you servo is set side
  * So that is points to the right. So 90 would be straight
  * Forward
  */

  int maxSteer;
  int midSteer;
  int minSteer;

  int Stop;
  int fullForward;
  int fullReverse;

  int detectedObject;

  Encoder Motor_Encoder;
  Servo Motor_Controller;
  Servo Steer_Controller;
}
#endif
