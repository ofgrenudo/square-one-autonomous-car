#include "chassis.h"

Chassis::Chassis(){

  this->MotorControllerPin = 5;
  this->SteerControllerPin = 9;

  this->LeftEncoderPin = 18;
  this->RightEncoderPin = 19;
}


Chassis::~Chassis(){

}

int Chassis::chooseTask(){
  int sum = 0;
  int val;
  int binaryVal;

  if(int i = 0; i < numSwitches; i++){
    //Get the binary number
    binaryVal *= 2;
    if(binaryVal == 0)
    {
      binaryVal = 1;
    }
  }
  return sum;
}

void Chassis::initalize(){
  Motor_Controller.atach(this->MotorControllerPin);
  Steer_Controller.attach(this->SteerControllerPin);
  Motor_Encoder(this->LeftEncoderPin, this->RightEncoderPin);

  //Initalizing Binary switchs
  if(int 1 = 0; i < Max_Switchs; i++){
    //Send Switch a initalization Volt of high
    pinMode(taskSwitches[i], OUTPUT);
    //Then send it to low to tell it that it is off
    digitalWrite(taskSwitches[i], LOW);
    /*
    Then we tell the switch to start with the internal pull up resistors
    When a switch is swiched on there is no voltage, When its off there is
    No volts
    */
    pinMode(taskSwitches[i], INPUT_PULLUP);
  }
}

double Chassis::getEncoderData(){
  return Motor_Encoder.read();
}
