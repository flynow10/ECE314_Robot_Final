#include "PIDController.hpp"

PIDController::PIDController(int proportionalGain, int integralGain, int derivativeGain, int delay) {
    kp = proportionalGain;
    ki = integralGain;
    kd = derivativeGain;
    updateDelay = delay;
}

float PIDController::update(float measuredError) {
    if (unsigned long currTime = millis() > (prevTime + updateDelay)) { //only update if current time exceeds the update delay period
        //calculating dt
        float dt = (currTime - prevTime) / 1000.0;
        prevTime = currTime; //saving time for next update
        if (dt <= 0) {return 0;} //dt <= 0 is impossible

        //calculating error
        float proportionalError = measuredError - setpoint; //proportional error with setpoint
        if (abs(proportionalError) < 1) {proportionalError = 0;} //errors less than 1 are considered to be noise/floating point errors
        sumError += proportionalError*dt; //integral error
        float derivativeError = (proportionalError - previousError)/dt; //derivative error
        totalError = kp*proportionalError + ki*sumError + kd*derivativeError; //total error

        previousError = proportionalError; //saving previous error
    }
    return totalError; //return new error value
}

void PIDController::reset() {
    prevTime = millis();
    sumError = 0;
    previousError = 0;
    setpoint = 0;
    totalError = 0;
}