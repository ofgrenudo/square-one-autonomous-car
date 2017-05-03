#include "chassis.h"

Chassis::Chassis(){

  MotorController = 5;
  SteerController = 9;

  LeftEncoder = 18;
  RightEncoder = 19;
}

Chassis::initalize(){
  Encoder Motor_Encoder(LeftEncoder, RightEncoder);
  Servo Motor_Controller(MotorController);
}

Chassis::~Chassis(){
  
}
