#include <Servo.h>
#include <Encoder.h>

// Takes about 1.5 seconds to turn from left to right

Servo steering;
Servo chassis;
Encoder enc(4, 5);

int steering_Pin = 1;
int driving_Pin = 3;

int task_3 = A1;
int task_2 = A2;
int task_1 = A3;

// Steering Positions
double LEFT = 55;
double HOME = 125;
double RIGHT = 185;

// DRIVING PWM
double MIN_FORWARD = 120;
double FORWARD = 130;
double MAX_FORWARD = 150;

double STOP = 90;

double MIN_REVERSE = 70;
double REVERSE = 60;
double MAX_REVERSE = 50;

// Encoder
int encoder_tick = 0;
int inches_traveled = 0;
int revolution_inch = 36;
int revolution_tick = 1100;
int inch_tick = 30;
int meter_tick = 800;

// Distance Sensors
int front_dist = A0;
long front_raw;
long front_mm, front_inches;

int STEP = 1;
int GENERIC_SPEED = 0.0;
int GENERIC_COUNTER = 0;

void drive(float speed){
  speed = 90*speed;
  chassis.write(90+speed);
}

void read_sensor (){
  front_raw = pulseIn(front_dist, HIGH);
  front_mm = front_raw - 32; //Takes the pulse width and tells Arduino it is equal to millimeters
  front_inches = front_mm/25.4; //Takes mm and converts it to inches 
//  Serial.print("S1");
//  Serial.print("=");
//  Serial.print(front_mm);
//  Serial.print(" ");
//  Serial.println(front_inches);
}

void task_one() {
  if(STEP == 1){
    if(encoder_tick < meter_tick*3){
      Serial.print("[STEP ONE] Meters Traveled : ");
      Serial.print(encoder_tick / meter_tick);
      Serial.println(" ");
      drive(0.15);
    } else {
      STEP = STEP + 1; 
    }    
  }
 
  if(STEP == 2){
    encoder_tick = 0;
    enc.write(0);
    STEP = STEP + 1;
  }
  
  if(STEP == 3){
    if(encoder_tick < meter_tick*1.1){
      steering.write(RIGHT);
      Serial.print("[STEP THREE] Meters Traveled : ");
      Serial.print(encoder_tick / meter_tick);
      Serial.println(" ");
      drive(0.15);
    } else {
      drive(0.0);
      STEP = STEP + 1; 
    } 
  }

  if(STEP == 4){
    steering.write(HOME);
    encoder_tick = 0;
    enc.write(0);
    STEP = STEP + 1;
  }

  if(STEP == 5){
    if(encoder_tick < meter_tick*3){
      Serial.print("[STEP FIVE] Meters Traveled : ");
      Serial.print(encoder_tick / meter_tick);
      Serial.println(" ");
      drive(0.15);
    } else {
      drive(0.0);
      STEP = STEP + 1; 
    }    
  }
}

void task_two(){
  steering.write(HOME);
  
  if(front_inches > 200){
    drive(0.2);

    if(front_inches > 300) {
      drive(0.3);
    }

    if(front_inches > 400) {
      drive(0.4);
    }
    
    if(front_inches > 500) {
      drive(0.3);
      if(GENERIC_COUNTER > 50){
        drive(0.4);
        if(GENERIC_COUNTER > 100){
          drive(0.5);
          if(GENERIC_COUNTER > 150){
            drive(0.9);        
          }
        }
      }
      GENERIC_COUNTER = GENERIC_COUNTER + 1;
      Serial.println(GENERIC_COUNTER);
    } else {
      GENERIC_COUNTER = 0;
    }

  } else {
    drive(0.0);
  }

  Serial.println(front_inches);
}

void command_chooser(){

}

void setup() {
  Serial.begin(9600);
  
  // Clear Encoder
  enc.write(0);
  encoder_tick = 0;
  inches_traveled = 0;
  
  steering.attach(steering_Pin);
  chassis.attach(driving_Pin);

  steering.write(HOME);

}



void loop() { 
  update_interfaces();
  int task_one_raw = analogRead(A3);
  int task_two_raw = analogRead(A2);
  int task_three_raw = analogRead(A1);
  int start_raw = analogRead(A5);

  float task_oneV = task_one_raw * (5.0 / 1023.0);
  float task_twoV = task_two_raw * (5.0 / 1023.0);
  float task_threeV = task_three_raw * (5.0 / 1023.0);
  float startV = start_raw * (5.0 / 1023.0);

  if(startV > 3) {
    if (task_oneV > 3){
      task_one();
    }
    if (task_twoV > 3){
      task_two();
    }
    if (task_threeV > 3){
      Serial.println("Task 3 Selected");
    }  
  } else {
    drive(0.0);
    steering.write(HOME);
  }
}

void update_interfaces(){
  encoder_tick = enc.read();
  read_sensor();
}
