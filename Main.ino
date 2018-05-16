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

int front_sensor = A1;
int back_sensor = A2;
int left_sensor = A3;
int right_sensor = A4;


// Interface Values
double encoder_tick;

// Steering Positions
double LEFT = 85;
double HOME = 100;
double RIGHT = 125;

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
double ENCODER_METER = ENCODER_INCH*39.5;

boolean DEBUGGING = true;

// Constants

int task = 2;
int mytask = 1;

// PID
double P = 0.01888;
double I = 0.0;
double D = 0.0;
float output_speed; 


int pulse;

int front_detection;
int back_detection;
int right_detection;
int left_detection;

Servo steering;
Servo chassis;
Encoder enc(encoder_left, encoder_right);

void setup() {
  // put your setup code here, to run once:
  // Attaching to servo
  steering.attach(turn_pin);
  chassis.attach(drive_pin);

  steering.write(HOME);

  enc.write(0);
  
  I2C.begin();


  Serial.begin(9600);

//  left_sensor.setADSampleDealy(10);
  
  delay(1000);
  Serial.println("1 Seconds");
  delay(1000);
  Serial.println("2 Seconds");
  delay(1000);
  Serial.println("3 Seconds");
  delay(1000);
  Serial.println("Starting...\n");
}

void loop() {
  update_interface();

  switch(task){
    case 1:
      task_one();
      break;
    case 2:
      task_two();
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
      if(encoder_tick <= ENCODER_METER*3){
        drive(0.3);
      } else {
        drive(0.0);
        mytask++;
      }  
      break;
    case 2:
      Serial.println("Clearing Encoder");
      encoder_tick = 0;
      steering.write(RIGHT);
      delay(2000);
      enc.write(0);
      mytask++;
      break;
    case 3:
      if(encoder_tick <= ENCODER_METER*5.4){
        drive(0.3);
      } else {
        steering.write(HOME);
        drive(0.0);
        delay(1500);
        mytask++;
      }
    pid_manager(encoder_tick, ENCODER_METER*5.4);
   }
}

void task_two(){
  Serial.println("Front [" + String(front_detection) + "]");
  Serial.println("Front [" + String(back_detection) + "]");
  Serial.println("Front [" + String(left_detection) + "]");
  Serial.println("Front [" + String(right_detection) + "]");
}

void update_interface(){
  encoder_tick = enc.read();

  if(DEBUGGING) Serial.println("Updating Distance Sensors"); 
  pinMode(front_sensor, INPUT);     
  pulse = pulseIn(front_sensor, HIGH);
  front_detection = pulse/147;

  pinMode(back_sensor, INPUT);  
  pulse = pulseIn(back_sensor, HIGH);
  back_detection = pulse/147;

  pinMode(left_sensor, INPUT);  
  pulse = pulseIn(left_sensor, HIGH);
  left_detection = pulse/147;

  pinMode(right_sensor, INPUT);
  pulse = pulseIn(right_sensor, HIGH);
  right_detection = pulse/147;
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

