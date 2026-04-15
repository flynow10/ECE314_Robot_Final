#ifndef PID 
#define PID

#include <Arduino.h>

class PIDController {
    private:
        int kp; //proportional gain
        int ki; //integral gain
        int kd; //derivative gain

        int updateDelay; //update delay
        unsigned long prevTime = 0; //previous time
        
        float totalError = 0; //post-PID error value
        
        float previousError = 0; //previous measured error value
        float sumError = 0; //integral error sum

    public:
        long setpoint = 0; //setpoint
        
        PIDController(int proportionalGain, int integralGain, int derivativeGain, int delay); //constructor
        float update(const float measuredError); //update the controller
        void reset(); //reset the controller
};

#endif