
#include <Maxbotix.h>
#include <Encoder.h>
#include "NAxisMotion.h"        //Contains the bridge code between the API and the Arduino Environment
#include <Wire.h>
#include <Servo.h>
#include <SPI.h>  
#include <Pixy.h>

NAxisMotion motionShield;         //Object that for the sensor 

Pixy pixy;//Pixy cam object

//Encoder pin numbers
int encA = 18;
int encB = 19;

Encoder enc(encA, encB);

//motor controllers and steer servo pin numbers
int drivePin = 5;
int steerPin = 9;
int frSweepPin = 12;

Servo motor;//Both motors that drive the jeep
Servo steer;//Servo that turns the steering
Servo frSweep;//Servo the front sensor is attached to

//Front Distance Sensor
int frSensorPin = 3;
Maxbotix frSensor(frSensorPin, Maxbotix::PW, Maxbotix::LV, Maxbotix::BEST);

/*
 * convert feet to inches
 * @param number of feet
 * @return inches
 */
int feetToInches(int feet)
{
  return feet * 12;
}

//Measurements of steering in inches
int rightTurnStart = 96;//48 + 6;//1.372 meters
int rightTurnEnd = 60;//108 + 6;//2.8976 meters
int leftTurnStart = 72;//1.067 meters
int leftTurnEnd = 67;//

//Range of steering
int maxSteerVal = 150;
int midSteerVal = 90;
int minSteerVal = 35;

//Range of motor speeds
int motorStop = 90;
int motorFullForward = 179;
int motorFullReverse = 8;

bool firstRun = true;
bool debugMode = true;

int task;//Holds the code to be run

// Binary code for selecting with task to run. 0=nothing
int taskSwitches[] = {6,7,8};
int numSwitches = 3;

//Lights
int headLights = 16;
int tailLights = 15;

void setup() {
  // put your setup code here, to run once:

  pinMode(headLights, OUTPUT);
  pinMode(tailLights, OUTPUT);
  digitalWrite(headLights, HIGH);
  digitalWrite(tailLights, HIGH);
  //Assign pins to servo
  motor.attach(drivePin);
  steer.attach(steerPin,700,2300);
  frSweep.attach(frSweepPin);
  
  Serial.begin(9600);
  Serial.println("Serial comm begin");

  pixy.init();//Init pixy cam
  
  I2C.begin();//Initialize I2C communication to the let the library communicate with the sensor.
  //Sensor Initialization
  motionShield.initSensor();          //The I2C Address can be changed here inside this function in the library
  motionShield.setOperationMode(OPERATION_MODE_NDOF);   //Can be configured to other operation modes as desired
  motionShield.setUpdateMode(MANUAL);  //The default is AUTO. Changing to MANUAL requires calling the relevant update functions prior to calling the read functions
  //Setting to MANUAL requires fewer reads to the sensor
  
  //Set up binary switches
  for(int i = 0; i < numSwitches; i++)
  {
    //Set output of switches to low first
    pinMode(taskSwitches[i], OUTPUT);
    digitalWrite(taskSwitches[i],LOW);
    //Then set the pins to be inputs with the internal pull up resistors on.
    //So when a switch is switched on, there's no voltage. When a switch is off, there's voltage.
    pinMode(taskSwitches[i], INPUT_PULLUP);
    Serial.print("Button on: ");
    Serial.print(taskSwitches[i]);
    Serial.print(" = ");
    Serial.println(digitalRead(taskSwitches[i])==HIGH);
  }
  
  if (debugMode) {
    Serial.println("IN DEBUGGING MODE");
  }

  //check dip switchs seting to determine which task to run.
  task = chooseTask();

  if (debugMode) {
    Serial.println("Task:");
    Serial.println(task); 
  }
  
  //Delay 5 seconds before running code
  delay(5000);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  if(firstRun)//Run only once
  {
    digitalWrite(headLights, LOW);
    digitalWrite(tailLights, LOW);
    switch(task)//Run code depending on task selected.
    {
      case 1:
        performTaskOne();
        break;
      case 2:
        performTaskTwo();
        break;
      case 3:
        performTaskThree(); 
        break;
      case 4:
        performTaskFour();
        break;
      case 5:
        performTaskFive();
        break;
      case 6:
        performTaskSix();
        break;
      case 7:
        performTaskSeven();
        break;
      case 8:
        performTaskEight();
        break;
      case 9:
        performTaskNine();
        break;
      default:
        Serial.println("Invalid input: ");
        Serial.println(task);
        break;
    }
    Serial.println("TASK DONE");
  }
  firstRun = false;
}

/*
 * Drive 3 meters, turn right, drive 3 meters
 */
void performTaskOne()
{
  steer.write(midSteerVal);
  drive(110, 78.740, 0);
  turn(90, 110, 0);
  delay(1000);
  turn(0, 110, 0);
  /*
  delay(2000);
  turn(90,110, 0);
  delay(2000);
  drive(110, 72, 0);
  */
}

void performTaskTwo()
{
  /*int newRightStart = 50;
  int newRightEnd = 112;
  //steer.write(midSteerVal);
  //drive(110,  396, 60);
  drive(110, 130 - newRightStart, 0);
  turn(90, 110, 0);
  drive(110, 180 - (newRightEnd), 0);
  //steer.write(minSteerVal);
  */
  drive(110,  396, 60);
}
/*
 * 20ft
 * turn right
 * 15ft
 * turn left
 * 16ft
 * turn left
 * 15ft
 * turn left
 * 36ft
 */
void performTaskThree()
{
  drive(110, (78.240 * 2) - leftTurnStart, 0);
  turn(-90, 110, 0);
  drive(110, ((78.240 * 3)+12) - (leftTurnEnd + rightTurnStart), 0);
  
  turn(90, 110, 0);
  drive(110, (78.240 * 2) - (rightTurnEnd + rightTurnStart), 0);
  turn(90, 110, 0);

  Serial.println("mess up");
  drive(110, ((78.240 * 2)) - (rightTurnEnd + leftTurnStart), 0);
  
  turn(-90, 110, 0);
  drive(110, ((78.240 * 2)+12) - (leftTurnEnd + leftTurnStart), 0);
  turn(-90, 110, 0);
  drive(110, ((78.240 * 2)+24) - (leftTurnEnd + rightTurnStart), 0);
  turn(90, 110, 0);
  drive(110, ((78.240 * 2)+12) - (rightTurnEnd + rightTurnStart), 0);
  turn(90, 110, 0);
  drive(110, ((78.240 * 2)+66) - (rightTurnEnd + leftTurnStart), 0);
  turn(-90, 110, 0);
  drive(110, 24, 0);
}

void performTaskFour()
{
  while(true)
  {
    drive(110, 200, 48);
  }
}
void performTaskFive()
{
  //drive(105, 110, 0);
  turnWithDistance(minSteerVal, 35, 80, 0);
  turnWithDistance(maxSteerVal, 35, 105, 0);
  turnWithDistance(minSteerVal, 10, 80, 0);
  turnWithDistance(maxSteerVal, 82, 80, 0);
  turnWithDistance(minSteerVal,15,105,0);  
  turnWithDistance(maxSteerVal, 20, 80, 0);
}
void performTaskSix()
{
  int green = 1;
  int red = 2;
  int yellow = 3;
  
  float initHeading = getScaledHeading(getRawHeading());
  float currentHeading = getScaledHeading(initHeading);
  float headingDiff = initHeading - currentHeading;
  int angle; 
  
  int motorSpeed = motorStop;
  
  uint16_t blocks;
  char buf[32]; 
  int strip;
  boolean done = false;
  
  while(!done)
  {
    currentHeading = getScaledHeading(initHeading);
    headingDiff = initHeading - currentHeading;
    headingDiff = headingDiff * 3;
    angle = midSteerVal + headingDiff;
    
    // grab blocks!
    blocks = pixy.getBlocks();
    
    // If there are detect blocks, print them!
    if (blocks)
    {      
      strip = processBlocks(blocks);
      Serial.print("Detected: ");
      Serial.println(strip);
      
      if(strip == green)
      {
        drive(110, 48, 0);
        delay(500);
      }
      else if(strip == red)
      {
        turn(90, 110, 0);
        //Get new orientation to keep
        initHeading = getScaledHeading(getRawHeading());
        delay(500);
      }
      else if(strip == yellow)
      {
        turn(-90, 110, 0);
        //Get new orientation to keep
        initHeading = getScaledHeading(getRawHeading());
        delay(500);
      }
      else
      {
        if(abs(motorSpeed - 110) > 0)
        {
          motorSpeed+=(2);  
        }
        else
        {
          motorSpeed = 110;
        }
      }
    }
    else
    {
       Serial.println("no blocks. driving forward");
        if(abs(motorSpeed - 110) > 0)
        {
            motorSpeed+=(2);  
        }
        else
        {
          motorSpeed = 110;
        }
    }
    motor.write(motorSpeed);
    steer.write(angle);
    delay(30);
  }
}
void performTaskSeven()
{
  digitalWrite(tailLights, HIGH);
  digitalWrite(headLights, HIGH);
  turn(-90, 110, 0);
  delay(200);
  turn(-90, 110, 0);
  delay(200);
  turn(-90, 110, 0);
  delay(200);
  turn(-90, 110, 0);
  delay(200);
  /*
  drive(110, 24, 30);
  delay(700);
  drive(80, 24, 0);
  delay(700);
  turn(45, 110, 0);
  delay(700);
  turn(0, 80, 0);
  delay(700);
  turn(-45, 110, 0);
  delay(700);
  turn(0, 80, 0);
  */
}
void performTaskEight()
{
   while(true)
   {
     Serial.println(enc.read());
   }
}
void performTaskNine()
{
  
}

/*
 * Move at a given speed to a given distance
 * @param speed: how fast the jeep should move. Range: 8(full reverse) - 179(full forward), 90(stop)
 * @param dist: the distance to travel in inches
 * @param detection: Jeep will only react/sense object below this distance(inches). Set this to zero if you don't want the jeep to react to anything.
 */

void drive(double targetSpeed, int dist, int detection)
{
  int dir;//The direction the motor will spin. -1 = reverse. 1 = forward
  int crawlSpeed;//Speed that the jeep will use to crawl to the target distance when it's near it
  
  //Determine direction to spin
  if(targetSpeed > motorStop)
  {
    crawlSpeed = 109;
    dir = 1;
  }
  else
  {
    crawlSpeed = 81;
    dir = -1;
  }
  
  //How much the jeep travels in one pulse from the encoder
  float inchPerPulse = 0.0285;
  //Calculate how many counts to travel to get to target distance
  int targetCount = dist/inchPerPulse;
  //Calculate the distance to start slowing down
  int intAccelerateDecelerateDistance = 8/inchPerPulse;//Distance before the target that the jeep will slow down and crawl

  int distance = getDistance(frSensorPin);//Distance read from the sensor
  
  int count = 0;//Holds encoder count
  enc.write(count);//Reset encoder to 0
  
  int motorSpeed = motorStop;//Start the motor at neutral
  
  float initHeading = getScaledHeading(getRawHeading());//Get init heading
  float currentHeading;
  float headingDiff;
  float angle = midSteerVal;

  Serial.print("Init heading: ");
  Serial.print(initHeading);
  Serial.print(" Target Count: ");
  Serial.println(targetCount);

  frSweep.write(90);
  
  while(count < targetCount)
  {
    //Update sensors
    distance = getDistance(frSensorPin);
    
    currentHeading = getScaledHeading(initHeading);
    headingDiff = initHeading - currentHeading;
    headingDiff = headingDiff * 3;
    
    angle = midSteerVal + (headingDiff*dir);//Angle to steer at

    //Slow down and stop if an object is in front
    if(distance < detection)
    {
      digitalWrite(tailLights, HIGH);
      digitalWrite(headLights, HIGH);
      while(abs(motorSpeed > motorStop))
      {
        motorSpeed+=(-dir*2);
        motor.write(motorSpeed);
        delay(30);
      }
      //Go around the object
      if(task!=2)
      {
        steerAroundObject();           
      }
    }
    //If there is no object in front, continue moving forward
    else
    {
      digitalWrite(tailLights, LOW);
      digitalWrite(headLights, LOW);
      //Accerelate to max speed and stay at max speed until jeep is a certain amount of distance away from target
      if(count < targetCount - intAccelerateDecelerateDistance)
      {
        if(abs(motorSpeed - targetSpeed) > 0)
        {
          motorSpeed+=(dir*2);  
        }
        else
        {
          motorSpeed = targetSpeed;
        }
      }
      //Go to a lower speed than the target speed. Jeep will "creep" up to the target distance.
      //Ensures that the jeep doesn't go past the target distance
      else if(count < targetCount)
      {
        if(motorSpeed > crawlSpeed)
        {
          motorSpeed -= 2;
        }
        else if(motorSpeed < crawlSpeed)
        {
          motorSpeed += 2;
        }
        else
        {
          motorSpeed = crawlSpeed;
        }
      }
    }
    
    //Debugging stuff
    Serial.print("Current heading: ");
    Serial.print(currentHeading);
    Serial.print(" Difference: ");
    Serial.print(headingDiff);
    Serial.print(" angle: ");
    Serial.print(angle);
    Serial.print(" enc: ");
    Serial.print(count);
    Serial.print(" speed: ");
    Serial.print(motorSpeed);
    Serial.print(" diff speed: ");
    Serial.print(abs(motorSpeed - targetSpeed ));
    Serial.print(" Distance: ");
    Serial.println(distance);

    //Write values to motors and steering
    steer.write(angle);
    motor.write(motorSpeed);
    
    //Update encoder readings
    count = abs(enc.read());
    delay(30);
  }
  
  motor.write(motorStop); // stop motor
  Serial.println("DONE MOVING");
}

/*
 * Turn jeep to angle
 * @param targetAngle: the angle to turn the jeep to
 * @param targetSpeed: the speed to spin the jeep at
 * @param detection: Jeep will only react/sense object below this distance(inches). Set this to zero if you don't want the jeep to react to anything.
 */
void turn(int targetAngle, int targetSpeed, int detection)
{
  int dir;
  int crawlSpeed;//Speed that the jeep will use to crawl to the target distance when it's near it
  
  //Determine direction to spin
  if(targetSpeed > motorStop)
  {
    crawlSpeed = 105;
    dir = 1;
  }
  else
  {
    crawlSpeed = 85;
    dir = -1;
  }
  
  int steerAngle;
  float initHeading = getRawHeading();
  float currentTurnHeading = getScaledHeading(initHeading);
  float angleDiff = targetAngle - currentTurnHeading;//Holds difference between target angle and current angle
  int motorSpeed = motorStop;//Holds motor speed
  
  int decelAngle = 8;//Jeep will begin to decelerate and crawl toward the target angle when it is this far away from the angle
  int distance = getDistance(frSensorPin);

  //Turn to direction of target angle
  if(angleDiff > 0)
  {
    if(dir == 1)
    {
      steerAngle = maxSteerVal;    
    }
    else
    {
      steerAngle = minSteerVal;
    }
  }
  else
  {
    if(dir == 1)
    {
      steerAngle = minSteerVal;  
    }
    else
    {
      steerAngle = maxSteerVal;
    }
  }
  Serial.print("init turn angle: ");
  Serial.println(initHeading);
  
  steer.write(steerAngle);
  Serial.print("init steering angle: ");
  Serial.println(steerAngle);
  
  //Keep turning until difference between the target angle and gyro is less than 1
  while(abs(angleDiff) >= 6)
  {
    //Update heading and difference and distance sensor reading
    currentTurnHeading = getScaledHeading(initHeading);
    angleDiff = targetAngle - currentTurnHeading;
    distance = getDistance(frSensorPin);

    //Slow down and stop if an object is in front
    if(distance < detection)
    {
      while(abs(motorSpeed > motorStop))
      {
        motorSpeed-=(5*dir);
        motor.write(motorSpeed);
        delay(30);
      }
      steer.write(midSteerVal);
      delay(250);
      //drive back
      drive(80,10,0);
      //Turn steering wheel back to original position
      steer.write(steerAngle);
      delay(250);
    }
    //Turn
    else
    {
        //Turn at target speed until jeep is decelAngle away from the target angle
        if(abs(angleDiff) > decelAngle)
        {
          //Accerelate to max speed     
          if(abs(motorSpeed-targetSpeed) > 0)
          {
              motorSpeed+=(dir*2);  
          }
          //Maintain target speed
          else
          {
              motorSpeed = targetSpeed;
          }   
        }
        //Then crawl to target angle
        else
        {
          if(motorSpeed > crawlSpeed)
          {
            motorSpeed -= 2;
          }
          else if(motorSpeed < crawlSpeed)
          {
            motorSpeed += 2;
          }
          else
          {
            motorSpeed = crawlSpeed;
          }
        }
    }

    //Debug
    Serial.print(" Target angle: ");
    Serial.print(targetAngle);    
    Serial.print("Current heading: ");
    Serial.print(currentTurnHeading);
    Serial.print(" Difference: ");
    Serial.print(abs(angleDiff));
    Serial.print(" Steering Angle: ");
    Serial.print(steerAngle);
    Serial.print(" speed: ");
    Serial.println(motorSpeed);

    motor.write(motorSpeed);
    delay(30);
  }
  
  Serial.print("Final heading: ");
  Serial.println(getRawHeading());
  //Stop motor and center the steering
  motor.write(motorStop);
  steer.write(midSteerVal);
}

/*
 * Similar to the drive command, except drive with the steering wheel at an angle
 * @param angle: the angle to turn the steering wheel at
 * @param dist: the distance to travel
 * @param targetSpeed: Speed to travel at
 * @param detection: Jeep will only react/sense object below this distance(inches). Set this to zero if you don't want the jeep to react to anything.
 */
void turnWithDistance(int angle, int dist, double targetSpeed, int detection)
{
  int dir;//The direction the motor will spin. -1 = reverse. 1 = forward
  int crawlSpeed;

  //Determine direction to spin
  if(targetSpeed > motorStop)
  {
    crawlSpeed = 105;
    dir = 1;
  }
  else
  {
    crawlSpeed = 85;
    dir = -1;
  }

  //How much the jeep travels in one pulse from the encoder
  float inchPerPulse = 0.0285;
  //Calculate how many counts to travel to get to target distance
  int targetCount = dist/inchPerPulse;
  //Calculate the distance to start slowing down
  int intAccelerateDecelerateDistance = 5/inchPerPulse; // distance needed to accelerate & decelerate in inch
  int distance = getDistance(frSensorPin);//Distance read from the sensor
  
  int count = 0;//Holds encoder count
  enc.write(count);//Reset encoder to 0
  
  int motorSpeed = motorStop;//Start the motor at neutral
  
  Serial.print(" Target Count: ");
  Serial.println(targetCount);

  frSweep.write(90);
  steer.write(angle);//Angle to steer at
  delay(300);//Give some time for the steering wheel to turn
  
  while(count < targetCount)
  {
    //Update sensors
    distance = getDistance(frSensorPin);

    //Slow down and stop if an object is in front
    if(distance < detection)
    {
      while(abs(motorSpeed > motorStop))
      {
        motorSpeed+=(-dir*5);
        motor.write(motorSpeed);
        delay(30);
      }
      steer.write(midSteerVal);
      delay(250);
      //drive back
      drive(80,12,0);
      //Turn steering wheel back to original position
      steer.write(angle);
      delay(300);      
    }
    //If there is no object in front, continue moving forward
    else
    {
      //Accerelate to max speed
      if(count < targetCount - intAccelerateDecelerateDistance)
      {
        if(abs(motorSpeed - targetSpeed ) > 0)
        {
          motorSpeed+=(dir*2);  
        }
        else
        {
          motorSpeed = targetSpeed;
        }
      }
      //Go to crawl speed
      else
      {
        if(motorSpeed > crawlSpeed)
        {
          motorSpeed -= 3;
        }
        else if(motorSpeed < crawlSpeed)
        {
          motorSpeed += 3;
        }
        else
        {
          motorSpeed = crawlSpeed;
        }
      }
    }
    
    //Debugging stuff
    Serial.print(" enc: ");
    Serial.print(count);
    Serial.print(" speed: ");
    Serial.print(motorSpeed);
    Serial.print(" diff speed: ");
    Serial.print(abs(motorSpeed - targetSpeed ));
    Serial.print(" Distance: ");
    Serial.println(distance);

    //Write values to motors
    motor.write(motorSpeed);
    
    //Update encoder readings
    count = abs(enc.read());
    delay(30);
  }
  
  motor.write(motorStop); // stop motor
  Serial.println("DONE MOVING");
}

/*
 * Calculate the sum of the binary switches to select task.
 * @return the sum of switches
 */
int chooseTask()
{
  int sum = 0;
  int val;//Hold the current switch in loop
  int binaryVal = 0;//Binary number
  
  for(int i = 0; i < numSwitches; i++)
  {
    //Get the binary number
    binaryVal *= 2;
    if(binaryVal == 0)
    {
      binaryVal = 1;
    }
    
    Serial.print("Converted val: ");
    Serial.print(convertSwitch(taskSwitches[i]));
    Serial.print(" ");
    Serial.print("Binary number: ");
    Serial.print(binaryVal);
    Serial.print(" ");
    val = convertSwitch(taskSwitches[i]) * binaryVal;
    Serial.print("val: ");
    Serial.print(val);
    Serial.print(" ");
    Serial.print("at button: ");
    Serial.println(taskSwitches[i]);
    Serial.println("-------");
    sum += val;
  }
  Serial.println("SUM: ");
  Serial.println(sum);
  Serial.println("-------");
  return sum;
}

/*
 * convert switch reading to either be 1 or 0.
 * @param the pin that the switch is connected to
 * @return 1 if switch reads low. 0 if switch reads high
 */
int convertSwitch(int port)
{
  int val;
  if(digitalRead(port) == LOW)
  {
    val = 1; 
  } 
  else
  {
    val = 0;
  }
  return val;
}
/*
 * Update and receive heading data from the 9 axis motion shield
 * @return the raw heading value from the shield
 */
float getHeading()
{
  motionShield.updateEuler();  //Update the Euler data into the structure of the object
  return translateDegree(motionShield.readEulerHeading());//Return heading
}

float getRawHeading()
{
  motionShield.updateEuler();  //Update the Euler data into the structure of the object
  return motionShield.readEulerHeading();
}

/*
 * convert the range to be -90 to 90 at a specific angle where that angle will be 0.
 * @param the angle that will be the middle of the -90 to 90 range
 * @return scaled heading range that is from -90 to 90. The angle in the param will be 0
 */
float getScaledHeading(int initAngle)
{
  if(initAngle <= 90)
  {
    if(getRawHeading() > 275)
    {
      return (getRawHeading()-initAngle) - 360;
    }
    else
    {
      return getRawHeading()-initAngle;
    }
  }
  else if(initAngle >= 270)
  {
    if(getRawHeading() <= 90)
    {
      return (getRawHeading() - initAngle) + 360;
    }
    else
    {
      return getRawHeading() - initAngle;
    }
  }
  return getRawHeading() - initAngle;
}
/*
 * Convert heading readings to -90 to 90
 * @param the raw heading value from the shield
 * @return scaled heading valued -90 to 90
 */
float translateDegree(float heading)
{
  if(heading > 95)
  {
    return -(360 - heading);
  }
  return heading;
}

/*
 * Get the distance in inches from a distance sensor
 * @param sensorPin - the pin that sensor is on
 * @return the distance calculated in inches
 */
int getDistance(int sensorPin)
{
  int dist = 0;
  if(sensorPin == frSensorPin)
  {
    frSensor.getRange();
    frSensor.getSample();
    dist = frSensor.getSampleBest() * 0.394;
  }
  /* 
  //Distance is calculated as the voltage read on the pin divided by 2
  dist = analogRead(sensorPin)/2;
  //Calculation not perfect, add 6
  dist += 6;
  */
  //Cap distance value at 200in
  if(dist > 200)
  {
    dist = 200;
  }
  return dist;
}

void steerAroundObject()
{
  //scan right
  frSweep.write(45);
  delay(200);
  Serial.print("Distance on right: ");
  Serial.println(getDistance(frSensorPin));
  if(getDistance(frSensorPin) > 90)
  {
    //Put sensor back to the front.
    frSweep.write(90);
    turn(40,110,24);
    delay(100);
    turn(-25,110,24);
    delay(100);
    turn(-40,110,24);
    delay(100);
    turn(30,110,24);
    delay(250);
  }
  //Scan left if there is something to the right
  else
  {
    frSweep.write(160);
    delay(200);
    Serial.print("Distance on left: ");
    Serial.println(getDistance(frSensorPin));
    if(getDistance(frSensorPin) > 90)
    {
      //Put sensor back to the front.
      frSweep.write(90);
      turn(-30,110,24);
      delay(100);
      turn(25,110,24);
      delay(100);
      turn(35,110,24);
      delay(100);
      turn(-15,110,24);
      delay(250);;
    }
  }
  //Put sensor back to the front.
  frSweep.write(90);
}

/*
 * Get all the detected colors from the camera and find the color strips
 * @return the color's signature. 1 is green, 2 is red, and 3 is yellow. -1 if no color strips are found
 */
int processBlocks(int blockCount)
{
  int sign;//Holds the sign of the block
  int yPos;
  int width;
  for(int i = 0; i < blockCount; i++)
  {
    sign = pixy.blocks[i].signature;
    yPos = pixy.blocks[i].y;
    width = pixy.blocks[i].width;
    if(yPos > 140 && width > 50)
    {
      return sign;
    }
  }
  return -1;
}
