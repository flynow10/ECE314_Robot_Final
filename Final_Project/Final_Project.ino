/* ---------- Libraries ---------- */
#include <Servo.h>
#include <Wire.h> //Library for I2C communication
#include <LiquidCrystal_I2C.h> //Library for LCD
#include <IRremote.hpp> //Library for IR Remote
#include <VL53L0X.h> //Library for ToF sensor


/* ---------- Motor Parameters ---------- */
const int RSPD = 150;        //Right Wheel PWM
const int LSPD = 150;        //Left Wheel PWM

//Left Wheel
const int LWhFwdPin = 8;
const int LWhBwdPin = 9;
const int LWhPWMPin = 6;

//Right Wheel
const int RWhFwdPin = 11;
const int RWhBwdPin = 10;
const int RWhPWMPin = 5; 

//turn constants
const long softTurnConstant = 25;
const long hardTurnConstant = 20;

/* ---------- IR Remote Parameters ---------- */
const int IRPin = 12;
long decodeTime = 0;

/* ---------- Servo Parameters ---------- */
Servo ultraServo;  //servo object to control ultrasonic servo
const int ultraServoPin = 1; //servo pin

/* ---------- LCD Parameters ---------- */
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2);

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

VL53L0X rSensor;
VL53L0X lSensor;

/* ---------- PID Parameters ---------- */

//reset variable
bool resetPID = false;

//gains
const float kp = 10;
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
  Stop = 0,
  Forward = 1,
  Backward = 2,
  Right = 3
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
  IrReceiver.begin(IRPin, ENABLE_LED_FEEDBACK); //start the reciever

  //setting up encoder interrupts
  attachInterrupt(digitalPinToInterrupt(LWEncoderPin), leftWhlCnt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RWEncoderPin), rightWhlCnt, CHANGE);

  //setting up ToF sensors
  Wire.begin();
  pinMode(rXShut, OUTPUT);
  pinMode(lXShut, OUTPUT);
  digitalWrite(rXShut, LOW);
  digitalWrite(lXShut, LOW);
  //set left sensor parameters
  delay(20);
  digitalWrite(lXShut, HIGH);
  delay(20);
  lSensor.setAddress(0x30);
  lSensor.setTimeout(500);
  if (!lSensor.init())
  {
    Serial.println("Failed to detect and initialize left sensor!");
    while (1) {}
  }
  //set right sensor parameters
  delay(20);
  digitalWrite(rXShut, HIGH);
  delay(20);
  rSensor.setAddress(0x31);
  rSensor.setTimeout(500);
  if (!rSensor.init())
  {
    Serial.println("Failed to detect and initialize right sensor!");
    while (1) {}
  }
}

void loop() {
 
  /* ---------- Sensors and LCD ---------- */
  //sending trigger signal to US sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  //reading echo back
  distance = pulseIn(echoPin, HIGH) / 74 / 2;
  lcd.setCursor(0, 1); // Set the cursor on the third column and first row.
  lcd.print("                ");
  lcd.setCursor(0, 1); // Set the cursor on the third column and first row.
  lcd.print(distance); // Print the string "Hello World!"
  lcd.setCursor(3, 1);
  lcd.print(lSensor.readRangeSingleMillimeters()); //print left ToF sensor reading
  lcd.setCursor(8, 1);
  lcd.print(rSensor.readRangeSingleMillimeters()); //print left ToF sensor reading
  
  /* ---------- IR Reciever ---------- */
  //setting states
  if (IrReceiver.decode()) { //checking for input from IR
    decodeTime = micros();
    switch (IrReceiver.decodedIRData.decodedRawData) {
      case 0x0: break; //do nothing when a button is held down
      case 0xB946FF00: current_state = Forward; break;
      case 0xEA15FF00: current_state = Backward; break;
      case 0xBC43FF00: current_state = Right; break;
      default: current_state = Stop; break;
    }
    IrReceiver.resume();
  } else if (micros() > (decodeTime + 500000L)) {
    current_state = Stop;
  }

  /* ---------- State Machine ---------- */
  //behavior based on states
  switch (current_state) {
    case Stop: stopMoving(); break;
    case Forward: moveForward(); break;
    case Backward: moveBackward(); break;
    case Right: hardTurnRight(); break;
    default: current_state = Stop; break;
  }
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
  long oldcntrL = cntrL;
  long oldcntrR = cntrR;
  cntrL = 0;
  cntrR = 0;
  while (cntrL < (cntrR + softTurnConstant)) { 
    analogWrite(RWhPWMPin,0); 
    analogWrite(LWhPWMPin,LSPD); 
  }
  stopMoving();
  delay(1000); 
  cntrL=oldcntrL;
  cntrR=oldcntrR;
}

void hardTurnRight() {
  digitalWrite(LWhFwdPin,HIGH); //enable left wheel forward
  digitalWrite(LWhBwdPin,LOW);
  digitalWrite(RWhFwdPin,LOW);   
  digitalWrite(RWhBwdPin,HIGH); //enable right wheel backward
  long oldcntrL = cntrL;
  long oldcntrR = cntrR;
  cntrL = 0;
  cntrR = 0;
  while (cntrL + cntrR < (2 * hardTurnConstant)) { 
    analogWrite(RWhPWMPin,RSPD - 30);
    analogWrite(LWhPWMPin,LSPD - 30); 
  }
  stopMoving();
  delay(1000); 
  cntrL=oldcntrL;
  cntrR=oldcntrR;
}

void softTurnLeft() {
  digitalWrite(LWhFwdPin,LOW);   
  digitalWrite(LWhBwdPin,LOW);
  digitalWrite(RWhFwdPin,HIGH);   //enable only right wheel
  digitalWrite(RWhBwdPin,LOW);
  long oldcntrL = cntrL;
  long oldcntrR = cntrR;
  cntrL = 0;
  cntrR = 0;
  while (cntrR < (cntrL + softTurnConstant)) { 
    analogWrite(RWhPWMPin,RSPD);
    analogWrite(LWhPWMPin,0);
  }
  stopMoving();
  delay(1000);
  cntrL=oldcntrL;
  cntrR=oldcntrR;
}

void hardTurnLeft() {
  digitalWrite(LWhFwdPin,LOW); 
  digitalWrite(LWhBwdPin,HIGH); //enable left wheel backward
  digitalWrite(RWhFwdPin,HIGH); //enable right wheel forward 
  digitalWrite(RWhBwdPin,LOW); 
  long oldcntrL = cntrL;
  long oldcntrR = cntrR;
  cntrL = 0;
  cntrR = 0;
  while (cntrL + cntrR < (2 * hardTurnConstant)) { 
    analogWrite(RWhPWMPin,RSPD - 30);
    analogWrite(LWhPWMPin,LSPD - 30); 
  }
  stopMoving();
  delay(1000); 
  cntrL=oldcntrL;
  cntrR=oldcntrR;
}

void PID() {
  /* ---------- PID Controller ---------- */
  delay(20); //delay
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
