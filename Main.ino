#include <Encoder.h>
#include <Servo.h>

// Pins to read
int turn_pin = 13;
int drive_pin = 12;
int encoder_left = 6;
int encoder_right = 7;

// Steering Positions
int LEFT = 85;
int HOME = 100;
int RIGHT = 115;

// Drive Speeds
int FORWARD;
int MID_FORWARD;
int REVERSE;
int MID_REVERSE;

// Task Selector
String input;
String task;
String status;

double ENCODER_INCH = 36.69;
double ENCODER_FOOT = ENCODER_INCH*12;

// Objects
Servo steering;
Servo chassis;

Encoder enc(encoder_left, encoder_right);


void setup() {
  // Attaching to servo
  steering.attach(turn_pin);
  chassis.attach(drive_pin);

  // Debug
  Serial.begin(9600);
  
  // Serial Communication
  
  // Reseting Encoder
  enc.write(0);
} 

void loop() {
  // Initialize
  init();

  // Task Selector
  
  // Task Executor
  switch (task) {
    case 'TASK_ONE':
      if(status) task_one();
      break;
    case 'TASK_TWO':
      if(status) task_two();
      break;
    case 'TASK_THREE':
      if(status) task_three();
      break;
    case 'TASK_FOUR':
      if(status) task_four();
      break;
    case 'TASK_FIVE':
      if(status) task_five();
      break;
    case 'TASK_SIX':
      if(status) task_six();
      break;
    case 'TASK_SEVEN':
      if(status) task_seven();
      break;
    case 'TASK_EIGHT':
      if(status) task_eight();
      break;    
  }
}

void init() {
  steering.write(HOME);
  Serial.println(enc.read());
}


void task_one(){
  if(enc.read() < ENCODER_FOOT*9){
    chassis.write(140);
  }
}



