/* ---------- Libraries ---------- */
#include <Servo.h>
#include <Wire.h> //Library for I2C communication
#include <LiquidCrystal_I2C.h> //Library for LCD
#include <IRremote.hpp> //Library for IR Remote
#include "ToF.hpp" //wrapper class for ToF sensor


/* ---------- Motor Parameters ---------- */
const int RSPD = 100;        //Right Wheel PWM
const int LSPD = 100;        //Left Wheel PWM

//Left Wheel
const int LWhFwdPin = 10;
const int LWhBwdPin = 11;
const int LWhPWMPin = 6;

//Right Wheel
const int RWhFwdPin = 13;
const int RWhBwdPin = 12;
const int RWhPWMPin = 5; 

//turn constants
const int softTurnConstant = 25;
const int hardTurnConstant = 15;

/* ---------- IR Remote Parameters ---------- */
const int IRPin = 8;

/* ---------- Servo Parameters ---------- */
Servo ultraServo;  //servo object to control ultrasonic servo
const int ultraServoPin = 9; //servo pin

/* ---------- LCD Parameters ---------- */
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2);
long updateTime = 0;

/* ---------- Rotery Encoder Parameters ---------- */
//pins
const int LWEncoderPin = 3;
const int RWEncoderPin = 2;

//interrupt variables
volatile long cntrL, cntrR;
volatile long LIntTime, RIntTime;

/* ---------- Ultrasonic Sensor Parameters ---------- */
const int trigPin = 4;
const int echoPin = 7;

long distance = 0;

/* ---------- ToF Sensor Parameters ---------- */
const int rXShut = A3;
const int lXShut = A2;

ToF rSensor = ToF(rXShut);
ToF lSensor = ToF(lXShut);

/* ---------- PID Parameters ---------- */

//reset variable
bool resetPID = false;

//gains
const float kp = 8;
const float ki = 1;
const float kd = 0.2;

//previous time
unsigned long prevTime = 0;

//persistant errors
float prevErr = 0;
float sumErr = 0;

//PID PWM variables
int PIDRSPD = 0; //Right Wheel PWM with PID control
int PIDLSPD = 0; //Left Wheel PWM with PID control

/* ---------- State Machine Parameters ---------- */
enum State {
  Stop,
  Start,
  Forward,
  Avoid_Obstacle
};

enum State current_state = Stop;

/* ---------- Program ---------- */

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
  lcd.print("US lToF rToF"); // Print the label

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
  if (millis() > updateTime + 100) {
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
    updateTime = millis();
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
  resetPID = true;
}

//move forward with PID
void moveForward() {
  digitalWrite(LWhFwdPin,HIGH);    //run left wheel forward
  digitalWrite(LWhBwdPin,LOW);
  digitalWrite(RWhFwdPin,HIGH);   //run right wheel forward
  digitalWrite(RWhBwdPin,LOW);
  PID();
  analogWrite(RWhPWMPin, PIDRSPD); //adjust speeds based on PID control
  analogWrite(LWhPWMPin, PIDLSPD);
}

//move backward with PID
void moveBackward() {
  digitalWrite(LWhFwdPin,LOW);    //run left wheel backward
  digitalWrite(LWhBwdPin,HIGH);
  digitalWrite(RWhFwdPin,LOW);    //run right wheel backward
  digitalWrite(RWhBwdPin,HIGH);
  PID();
  analogWrite(RWhPWMPin, PIDRSPD); //adjust speeds based on PID control
  analogWrite(LWhPWMPin, PIDLSPD);
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

void PID() {
  /* ---------- PID Controller ---------- */
  delay(10); //delay
  //reset code
  if (resetPID) {
    resetPID = false;
    cntrL = 0;
    cntrR = 0;
    prevTime = millis();
    sumErr = 0;
    prevErr = 0;
  }

  //getting counter values
  long tmpLcntr = cntrL;
  long tmpRcntr = cntrR;

  //calculating dt
  unsigned long currTime = millis();
  float dt = (currTime - prevTime) / 1000.0;
  prevTime = currTime;
  if (dt <= 0) {return;} //dt <= 0 is impossible

  //calculating error
  float propErr = tmpLcntr - tmpRcntr; //proportional error
  if (abs(propErr) < 1) {propErr = 0;} //errors less than 1 are impossible
  sumErr += propErr*dt; //integral error
  float derErr = (propErr - prevErr)/dt; //derivative error
  float totalErr = kp*propErr + ki*sumErr + kd*derErr; //total error

  Serial.println(totalErr);
  
  //adjusting control values
  const int PID_MAX = 50; //maximum modification of motor speed from PID control
  PIDLSPD  = constrain(LSPD - totalErr, LSPD - PID_MAX, LSPD + PID_MAX);
  PIDRSPD = constrain(RSPD + totalErr, RSPD - PID_MAX, RSPD + PID_MAX);
  
  prevErr = propErr; //saving previous error
}
