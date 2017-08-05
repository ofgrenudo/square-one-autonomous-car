#include <Encoder.h>
#include "NAxisMotion.h"
#include <Wire.h>
#include <Servo.h>
#include <SPI.h>
#include <Pixy.h>

//Objects
NAxisMotion motionShield;
Pixy camera;
Servo motor;
Servo steer;
Servo Sweeper;

//Motors
//Pin numbers for ease of access
int encLeftPin = 18;
int encRightPin = 19;
int drivePin = 5;
int steerPin = 9;
int frSweeperPin = 12;
int frSensorPin = 3;

//Paramaters
//Some commonly used steering values
int rightTurnStart = 96;
int rightTurnEnd = 60;
int leftTurnStart = 72;
int leftTurnEnd = 67;

//Values for the steering motor
//Remember that it is sideways so you
int maxSteer = 150;
int midSteer = 90;
int minSteer = 35;

//Debuging Data
bool firstRun = true;
bool debugMode = true;

//What task to run
int task;

//Binary Code to run for selectin which task to run
//0 = nothing
int taskSwitches[] = {
    6,
    7,
    8
}

//Bling
int headLights = 16;
int tailLights = 15;

//Max Switches on board
int switches = 3;

Matbotix frSensor(frSensorPin, Maxbotix::PW, Maxbotix::LV, Maxbotix::BEST);
Encoder enc(encLeftPin, encRightPin);


int feetToInches(int feet){
    return feet * 12;
}

void setup(){
    pinMode(headLights, OUTPUT);
    pinMode(tailLights, OUTPUT);
    digitalWrite(headLights, HIGH);
    digitalWrite(tailLights, HIGH);

    motor.attach(drivePin);
    steer.attach(steerPin, 700, 2300);
    frSweep.attach(frSweepPin);

    //Start serial communications at 9000 buad rate
    Serial.begin(9000);
    Serial.println("Serial Comm Begin");

    pixy.init();
    I2C.begin();
    motionShield.initSensor();
    motionShield.setOperationMode(OPERATION_MODE_NDOF);
    motionShield.setUpdateMode(MANUAL);

    for(int i = 0; i < numSwitches; i++){
        
    }
}
