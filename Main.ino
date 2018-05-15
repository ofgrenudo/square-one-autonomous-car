#include <Maxbotix.h>
#include <Encoder.h>
#include "NAxisMotion.h"        //Contains the bridge code between the API and the Arduino Environment
#include <Wire.h>
#include <Servo.h>
#include <SPI.h>  

NAxisMotion motionShield; 

// Pins to read
int turn_pin = 13;
int drive_pin = 12;
int encoder_left = 3;
int encoder_right = 4;

// Interface Values
double encoder_tick;

// Steering Positions
double LEFT = 85;
double HOME = 100;
double RIGHT = 115;

int Step = 1;

// Drive Speeds
double MIN_FORWARD = 72;
double FORWARD = 32;
double MAX_FORWARD = 0;

double STOP = 90;

double MIN_REVERSE = 100;
double REVERSE = 109;
double MAX_REVERSE = 250;

double ENCODER_INCH = 22.6696;
double ENCODER_FOOT = ENCODER_INCH*12;

boolean DEBUGGING = true;

int task = 1;
int mytask = 1;

// PID
double P = 0.008488;
double I = 0.0;
double D = 0.0;
float output_speed; 


Servo steering;
Servo chassis;
Encoder enc(encoder_left, encoder_right);

void setup() {
  // put your setup code here, to run once:
  // Attaching to servo
  steering.attach(turn_pin);
  chassis.attach(drive_pin);

  enc.write(0);

  Serial.begin(9600);
}

void loop() {
  update_interface();

  switch(task){
    case 1:
      task_one();
      break;
    case 2:
     Serial.println("Starting Task Two");
     break;
    case 3:
      Serial.println("Starting Task Three");
      break;
    case 4:
      Serial.println("Starting Task Four");
      break;
  }
}

void task_one(){
  switch(mytask){
    case 1:
      if(encoder_tick <= ENCODER_FOOT*3.3){
        drive(pid_manager(encoder_tick, ENCODER_FOOT*3.3));
      } else {
        drive(0.0);
        mytask++;
      }  
      break;
    case 2:
      Serial.println("\nLast Given Value \n");
      pid_manager(encoder_tick, ENCODER_FOOT*3.3);
      mytask++;
      break;
   }
}

void update_interface(){
  encoder_tick = enc.read();
}

void drive(float speed){
  speed = 90*speed;
  chassis.write(90-speed);
}

double pid_manager(double actual, double destination) {
  double Previous_Error, Setpoint, Actual, Error;
  double Output, Intergral, Derivative;

  // Setting Destination and Our Current Position
  Actual = actual;
  Setpoint = destination;
  
  Error = Setpoint - Actual;

  // Calculation our output
  Intergral += (Error*0.2);
  Derivative = (Error - Previous_Error) / 0.2;
  Output = P*Error + I*Intergral + D*Derivative;

  Output = Output/10;
  
  if(Error < Setpoint*.25) {
    Output = Output*.25;
  } else if(Error < Setpoint*.50) {
    Output = Output*.50;
  } else if(Error < Setpoint*.75) {
    Output = Output*.75;
  } else if(Error < Setpoint*.100) {
    Output = 0.0;
  } else if(Error > Setpoint) { 
    Output = 0.0;
  }

  // Limit Output to max 1
  if (Output > 1){
    Output = 1;
  }

  /*
   * If actual is greater than setpoint set 
   * output to 0.0 to stop the car.  This prevents osolation
   */
  if (Actual > Setpoint) {
   Output = 0.0;
  }

  if(DEBUGGING) Serial.println("Actual [" + String(Actual) + "] Destination [" + String(Setpoint) + "] Speed [" + Output + "]");
  return Output;
}

