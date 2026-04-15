/* ---------- Libraries ---------- */
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h> //Library for I2C communication
#include <LiquidCrystal_I2C.h> //Library for LCD
#include <IRremote.hpp> //Library for IR Remote
#include "ToF.hpp" //wrapper class for ToF sensor
#include "PIDController.hpp" //PID controller class


/* ---------- Motor Parameters ---------- */
constexpr int RSPD = 100;        //Right Wheel PWM
constexpr int LSPD = 100;        //Left Wheel PWM

//Left Wheel
constexpr int LWhFwdPin = 10;
constexpr int LWhBwdPin = 11;
constexpr int LWhPWMPin = 6;

//Right Wheel
constexpr int RWhFwdPin = 13;
constexpr int RWhBwdPin = 12;
constexpr int RWhPWMPin = 5; 

//turn constants
constexpr int softTurnConstant = 25;
constexpr int hardTurnConstant = 15;

/* ---------- IR Remote Parameters ---------- */
constexpr int IRPin = 8;

/* ---------- Servo Parameters ---------- */
Servo ultraServo;  //servo object to control ultrasonic servo
constexpr int ultraServoPin = 9; //servo pin

/* ---------- LCD Parameters ---------- */
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2);
unsigned long LCDUpdateTime = 0;

/* ---------- Rotery Encoder Parameters ---------- */
//pins
constexpr int LWEncoderPin = 3;
constexpr int RWEncoderPin = 2;

//interrupt variables
volatile long cntrL, cntrR;
volatile long LIntTime, RIntTime;

/* ---------- Ultrasonic Sensor Parameters ---------- */
constexpr int trigPin = 4;
constexpr int echoPin = 7;

long distance = 0;

/* ---------- ToF Sensor Parameters ---------- */
constexpr int rXShut = A3;
constexpr int lXShut = A2;

ToF rSensor(rXShut);
ToF lSensor(lXShut);

unsigned long wallFollowUpdateTime = 0;

constexpr int wallFollowDistance = 200;
constexpr int wallFollowMaxDelta = 10;
constexpr int wallFollowGain = 2;

int previousWallDistance = 0;

/* ---------- Motor PID Parameters ---------- */

//gains
constexpr float MOTOR_KP = 8;
constexpr float MOTOR_KI = 1;
constexpr float MOTOR_KD = 0.2;

constexpr int MOTOR_PID_UPDATE_TIME = 10;
constexpr int MOTOR_PID_MAX = 50; //maximum modification of motor speed from PID control

PIDController motorPID(MOTOR_KP, MOTOR_KI, MOTOR_KD, MOTOR_PID_UPDATE_TIME);

/* ---------- State Machine Parameters ---------- */
enum State {
  Stop,
  Start,
  Forward,
  Avoid_Obstacle
};

enum State current_state = Stop;

/* ---------- Program ---------- */

long ultraSonicRead() {
  //sending trigger signal to US sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  //reading echo back
  return pulseIn(echoPin, HIGH) / 74 / 2;
}

//left wheel ISR
void leftWhlCnt()
{
  long intTime = micros();
  if(intTime > LIntTime + 1000L)
  {
    LIntTime = intTime;
    cntrL++;
  }
}

//right wheel ISR
void rightWhlCnt()
{
  long intTime = micros();
  if(intTime > RIntTime + 1000L) {
    RIntTime = intTime;
    cntrR++;
  }
}

//stopping robot
void stopMoving() {
  digitalWrite(LWhFwdPin,LOW);   //stop all wheels
  digitalWrite(LWhBwdPin,LOW);
  digitalWrite(RWhFwdPin,LOW);   
  digitalWrite(RWhBwdPin,LOW);
  analogWrite(RWhPWMPin, 0);
  analogWrite(LWhPWMPin, 0);
  motorPID.reset();
  cntrL = 0; //counters should be reset when the robot stops moving
  cntrR = 0;
}

//move forward with PID and following right wall
void moveForward() {
  //writing pins
  digitalWrite(LWhFwdPin,HIGH);    //run left wheel forward
  digitalWrite(LWhBwdPin,LOW);
  digitalWrite(RWhFwdPin,HIGH);   //run right wheel forward
  digitalWrite(RWhBwdPin,LOW);

  //pid control for wall following
  if (millis() > wallFollowUpdateTime + 500) {
    int wallDist = rSensor.getRangeMilimeters();
    if (wallDist < (wallFollowDistance - wallFollowMaxDelta)) {
      motorPID.setpoint = motorPID.setpoint - wallFollowGain;
    } else if (wallDist > (wallFollowDistance + wallFollowMaxDelta)) {
      motorPID.setpoint = motorPID.setpoint + wallFollowGain;
    }
    wallFollowUpdateTime = millis();
  }

  //pid control for motor speed
  float error = motorPID.update(cntrL-cntrR);
  
  analogWrite(RWhPWMPin, constrain(RSPD + error, RSPD - MOTOR_PID_MAX, RSPD + MOTOR_PID_MAX)); //adjust speeds based on PID control
  analogWrite(LWhPWMPin, constrain(LSPD - error, LSPD - MOTOR_PID_MAX, LSPD + MOTOR_PID_MAX));
}

//move backward with PID
void moveBackward() {
  digitalWrite(LWhFwdPin,LOW);    //run left wheel backward
  digitalWrite(LWhBwdPin,HIGH);
  digitalWrite(RWhFwdPin,LOW);    //run right wheel backward
  digitalWrite(RWhBwdPin,HIGH);
  //pid control for motor speed
  float error = motorPID.update(cntrL-cntrR);
  analogWrite(RWhPWMPin, constrain(RSPD + error, RSPD - MOTOR_PID_MAX, RSPD + MOTOR_PID_MAX)); //adjust speeds based on PID control
  analogWrite(LWhPWMPin, constrain(LSPD - error, LSPD - MOTOR_PID_MAX, LSPD + MOTOR_PID_MAX));
}

void softTurnRight() {
  digitalWrite(LWhFwdPin,HIGH);   //enable only left wheel
  digitalWrite(LWhBwdPin,LOW);
  digitalWrite(RWhFwdPin,LOW);   
  digitalWrite(RWhBwdPin,LOW);
  cntrL = 0;
  cntrR = 0;
  while (cntrL < (cntrR + softTurnConstant)) { 
    analogWrite(RWhPWMPin,0); 
    analogWrite(LWhPWMPin,LSPD); 
  }
  stopMoving();
  delay(1000); 
}

void hardTurnRight() {
  digitalWrite(LWhFwdPin,HIGH); //enable left wheel forward
  digitalWrite(LWhBwdPin,LOW);
  digitalWrite(RWhFwdPin,LOW);   
  digitalWrite(RWhBwdPin,HIGH); //enable right wheel backward
  cntrL = 0;
  cntrR = 0;
  while (cntrL + cntrR < (2 * hardTurnConstant)) { 
    analogWrite(RWhPWMPin,RSPD);
    analogWrite(LWhPWMPin,LSPD); 
  }
  stopMoving();
  delay(1000); 
}

void softTurnLeft() {
  digitalWrite(LWhFwdPin,LOW);   
  digitalWrite(LWhBwdPin,LOW);
  digitalWrite(RWhFwdPin,HIGH);   //enable only right wheel
  digitalWrite(RWhBwdPin,LOW);
  cntrL = 0;
  cntrR = 0;
  while (cntrR < (cntrL + softTurnConstant)) { 
    analogWrite(RWhPWMPin,RSPD);
    analogWrite(LWhPWMPin,0);
  }
  stopMoving();
  delay(1000);
}

void hardTurnLeft() {
  digitalWrite(LWhFwdPin,LOW); 
  digitalWrite(LWhBwdPin,HIGH); //enable left wheel backward
  digitalWrite(RWhFwdPin,HIGH); //enable right wheel forward 
  digitalWrite(RWhBwdPin,LOW); 
  cntrL = 0;
  cntrR = 0;
  while (cntrL + cntrR < (2 * hardTurnConstant)) { 
    analogWrite(RWhPWMPin,RSPD);
    analogWrite(LWhPWMPin,LSPD); 
  }
  stopMoving();
  delay(1000);
}

//setup
void setup() {

  //setting up motor pins
  pinMode(LWhFwdPin,OUTPUT);
  pinMode(LWhBwdPin,OUTPUT);
  pinMode(LWhPWMPin,OUTPUT);
  pinMode(RWhFwdPin,OUTPUT);
  pinMode(RWhBwdPin,OUTPUT);
  pinMode(RWhPWMPin,OUTPUT);

  digitalWrite(LWhFwdPin,LOW);
  digitalWrite(LWhBwdPin,LOW);
  analogWrite(LWhPWMPin,LSPD);
  
  digitalWrite(RWhFwdPin,LOW);
  digitalWrite(RWhBwdPin,LOW);
  analogWrite(RWhPWMPin,RSPD);

  //setting up servo
  ultraServo.attach(ultraServoPin);

  //setting up ultrasonic sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  //setting up LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0); // Set the cursor on the third column and first row.
  lcd.print("US lToF rToF St"); // Print the label

  //setting up IR Remote
  pinMode(IRPin, INPUT);
  Serial.begin(9600);
  IrReceiver.begin(IRPin, DISABLE_LED_FEEDBACK); //start the reciever

  //setting up encoder interrupts
  attachInterrupt(digitalPinToInterrupt(LWEncoderPin), leftWhlCnt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RWEncoderPin), rightWhlCnt, CHANGE);

  //setting up ToF sensors
  lSensor.init(0x30);
  rSensor.init(0x31);
}

void loop() {
  /* ---------- Sensors and LCD ---------- */
  if (millis() > LCDUpdateTime + 100) {
    lcd.setCursor(0, 1); 
    lcd.print("                "); //clear bottom row
    lcd.setCursor(0, 1); 
    lcd.print(distance); // Print the current ultrasonic reading in inches
    lcd.setCursor(3, 1);
    lcd.print(lSensor.getRangeMilimeters()); //print left ToF sensor reading
    lcd.setCursor(8, 1);
    lcd.print(rSensor.getRangeMilimeters()); //print left ToF sensor reading
    lcd.setCursor(14, 1);
    lcd.print(current_state);
    LCDUpdateTime = millis();
  }
  /* ---------- IR Reciever ---------- */
  //setting states
  if (IrReceiver.decode()) { //checking for input from IR
    switch (IrReceiver.decodedIRData.decodedRawData) {
      case 0x0: break; //do nothing when a button is held down
      case 0xBA45FF00: current_state = Start; break;
      case 0xB847FF00: current_state = Stop; break;
      default: break;
    }
    IrReceiver.resume();
  }

  /* ---------- State Machine ---------- */
  //behavior based on states
  switch (current_state) {
    case Stop: { //robot stays stopped in the stop case
      stopMoving(); 
      current_state = Stop;
      break;
    }
    case Start: { //robot begins by moving forward
      current_state = Forward;
    }
    case Forward: { //robot moves forward unless it detects an obstacle
      moveForward();
      distance = ultraSonicRead();
      if (distance < 8) {
        current_state = Avoid_Obstacle;
      } else {
        current_state = Forward;
      }
      break;
    }
    case Avoid_Obstacle: { //checks for obstacles left and right and turns in a direction that is unobstructed
      stopMoving(); //stop the robot
      //measuring distances
      ultraServo.write(0);
      delay(1000);
      long left = ultraSonicRead();
      ultraServo.write(180);
      delay(1000);
      long right = ultraSonicRead();
      ultraServo.write(90);
      if ((left < 10) && (right < 10)) {
        hardTurnRight();
        hardTurnRight();
      } else if (left > right) {
        hardTurnRight();
      } else if (right > left) {
        hardTurnLeft();
      }
      current_state = Forward;
      break;
    }
    default: break;
  }
}
