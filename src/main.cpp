#include <Arduino.h>
// ReSharper disable once CppUnusedIncludeDirective
#include <Wire.h>
#include <IRremote.hpp>
#include <LiquidCrystal_I2C.h>
#include "pid.cpp"
#include "key.hpp"
#include "motor.hpp"
#include "pins.hpp"
#include "ultrasonic.hpp"
#include "utils.hpp"
#include "ToF.hpp"


// Setup constants

constexpr int RightBaseSpeed = 200;        //Right Wheel PWM
constexpr int LeftBaseSpeed = 250;        //Left Wheel PWM

constexpr int LeftTurnCounterOffset = 25;
constexpr int RightTurnCounterOffset = 25;

volatile long wheel_counter_left, wheel_counter_right;
volatile ulong LIntTime, RIntTime;

enum State {
  FORWARD,
  BACKWARD,
  TURN_LEFT,
  STOP
};

State currentState = STOP;

Motor leftMotor(LeftForwardPin, LeftBackwardPin, LeftPWMPin, LeftBaseSpeed);
Motor rightMotor(RightForwardPin, RightBackwardPin, RightPWMPin, RightBaseSpeed);
ToF lSensor(lSensorPin);
ToF rSensor(rSensorPin);
LiquidCrystal_I2C lcd(0x27, 16, 2);
PID<long> pidController(6, 3, 2);

void drive(const int speed) {
  int power[] = {speed, speed};
  const long error = wheel_counter_left - wheel_counter_right;
  pidController.update_error(error);
  const int output = static_cast<int>(pidController.get_output());
  if(output > 0) {
    power[0] -= min(output, power[0]);
  } else {
    power[1] -= min(-output, power[1]);
  }
  leftMotor.set_speed(power[0]);
  rightMotor.set_speed(power[1]);
  leftMotor.start();
  rightMotor.start();
}

void stop() {
  leftMotor.stop();
  rightMotor.stop();
}

void turnRight() {
  const long saveLeft = wheel_counter_left;
  const long saveRight = wheel_counter_right;
  wheel_counter_left = 0;
  wheel_counter_right = 0;

  while (wheel_counter_right < wheel_counter_left + RightTurnCounterOffset) {
    rightMotor.stop();
    leftMotor.set_speed(100);
  }
  wheel_counter_left = saveLeft;
  wheel_counter_right = saveRight;
}

void turnLeft() {
  const long saveLeft = wheel_counter_left;
  const long saveRight = wheel_counter_right;
  wheel_counter_left = 0;
  wheel_counter_right = 0;

  while (wheel_counter_left < wheel_counter_right + LeftTurnCounterOffset) {
    leftMotor.stop();
    rightMotor.set_speed(100);
  }
  wheel_counter_left = saveLeft;
  wheel_counter_right = saveRight;
}

void leftWhlCnt()
{
  const ulong intTime = micros();
  if(intTime > LIntTime + 1000L)
  {
    LIntTime = intTime;
    wheel_counter_left++;
  }
}

void rightWhlCnt()  // Complete this ISR for your right wheel
{
  const ulong intTime = micros();
  if(intTime > RIntTime + 1000L) {
    RIntTime = intTime;
    wheel_counter_right++;
  }
}

void setup() {

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0); // Set the cursor on the third column and first row.
  lcd.print("US lToF rToF St"); // Print the label

  // Initialize pins for motors
  leftMotor.init();
  rightMotor.init();

  initUltrasonic();

  pinMode(IRPin, INPUT);
  IrReceiver.begin(IRPin, DISABLE_LED_FEEDBACK);

  // Setup wheel encoder interrupts
  attachInterrupt(digitalPinToInterrupt(2), rightWhlCnt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), leftWhlCnt, CHANGE);

  lSensor.init(0x30);
  rSensor.init(0x31);

  wheel_counter_right = 0;
  wheel_counter_left = 0;
  LIntTime = 0;
  RIntTime = 0;
}

void loop() {
  lcd.setCursor(0, 1); 
  lcd.print("                "); //clear bottom row
  lcd.setCursor(0, 1); 
  lcd.print(microsecondsToInches(readUltrasonic())); // Print the current ultrasonic reading in inches
  lcd.setCursor(3, 1);
  lcd.print(lSensor.getRangeMilimeters()); //print left ToF sensor reading
  lcd.setCursor(8, 1);
  lcd.print(rSensor.getRangeMilimeters()); //print left ToF sensor reading
  lcd.setCursor(14, 1);
  lcd.print(currentState);
  // Serial.println(currentState);
  switch(currentState) {
    case STOP: {
      stop();
      break;
    }
    case FORWARD: {
      drive(100);
      /*
      const ulong inches = readUltrasonic();
      if(inches != 0 && inches <= 4) {
        currentState = TURN_LEFT;
      }
        */
      break;
    }
    case TURN_LEFT: {
      turnLeft();
      currentState = FORWARD;
      break;
    }
    case BACKWARD: {
      drive(-100);
      break;
    }
  }
  const auto key = tryReadKey();
  if(key != nullptr) {
    if(key->name == "Play") {
      currentState = STOP;
    } else if(key->name == "Prev") {
      currentState = BACKWARD;
    } else if(key->name == "Next") {
      currentState = FORWARD;
    } else {
      return;
    }
    wheel_counter_left = 0;
    wheel_counter_right = 0;
  }
}
