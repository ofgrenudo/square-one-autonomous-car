#ifndef CHASSIS_H
#define CHASSIS_H
class Chassis{
public:
  Chassis();
  virtual ~Chassis();

  void initalize;
private:
  //Motor Controller for both Left and Right
  int MotorController;
  //Steer Controller, The Driving System
  int SteerController;

  int LeftEncoder;
  int RightEncoder;
}
#endif
