#ifndef PID 
#define PID

#include <Arduino.h>

class PIDController {
    private:
        //gain values
        int kp;
        int ki;
        int kd;

        int updateDelay; //update delay
        unsigned long prevTime = 0; //previous time
        
        float totalError = 0;
        
        //persistant errors
        float previousError = 0;
        float sumError = 0;

    public:
        long setpoint = 0; //setpoint
        
        PIDController(int proportionalGain, int integralGain, int derivativeGain, int delay); //constructor
        float update(float error); //update the controller
        void reset(); //reset the controller
};

#endif