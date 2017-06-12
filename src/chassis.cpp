#include "chassis.h"

Chassis::Chassis(){

  this->MotorControllerPin = 5;
  this->SteerControllerPin = 9;

  this->LeftEncoderPin = 18;
  this->RightEncoderPin = 19;

  this->sweeperPin = 12;

  this->maxSteer = 150;
  this->midSteer = 90;
  this->minSteer = 35;

  this->Stop = 90;
  this->fullForward = 179;
  this->fullReverse = 8;

  this->detectedObject = 0;

}


Chassis::~Chassis(){

}


void Chassis::initalize(){
  Motor_Controller.atach(this->MotorControllerPin);
  Steer_Controller.attach(this->SteerControllerPin);

  Motor_Encoder(this->LeftEncoderPin, this->RightEncoderPin);
  pinMode(sweeperPin, INPUT);
  }
}

void drive(double speed, int distance, int detection){
  //Calculate Distane the jeep needs to travel
  float inchPulse = 0.0285;
  int targetCount = distance/inchPulse;
  int count = 0;
  Motor_Encoder.write(count);

  Motor_Controller.write(this->Stop);

  //Start Function loop
  while (count < targetCount) {
    Motor_Controller.write(speed);
    pulse = pulseIn(sweeperPin, HIGH);
    detectedObject = pulse/147
    if(detectedObject > detection){
      Motor_Controller.write(this->Stop);
      Motor_Controller.write(this->fullReverse);
    }
  }

  //Chase Mode Off
  Motor_Controller.write(this->Stop);
}

void turn(int angle, int distance, double speed, int detection){
  //Calculate Distane the jeep needs to travel
  float inchPulse = 0.0285;
  int targetCount = distance/inchPulse;
  int count = 0;
  Motor_Encoder.write(count);

  Steer_Controller.write(angle);
  delay(300);

  //Begin the loop
  while (count < targetCount) {
    Motor_Controller.write(speed);
    pulse = pulseIn(sweeperPin, HIGH);
    detectedObject = pulse/147
    if(detectedObject > detection){
      Motor_Controller.write(this->Stop);
      Motor_Controller.write(this->fullReverse);
      delay(100);
    }
  }

  //Chase mode off
  Motor_Controller.write(this->Stop);
  Steer_Controller.write(this->midSteer);
}

double Chassis::getEncoderData(){
  return Motor_Encoder.read();
}
