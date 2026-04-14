//
// Created by Natalie Wagner on 4/10/26.
//

#ifndef MOTOR_HPP
#define MOTOR_HPP
#include "digitalWriteFast.h"

class Motor {
    int forwards_pin;
    int backwards_pin;
    int pwm_pin;
    int pwm_base;
    int speed;
    bool moving;
public:
    Motor(int forward_pin, int backward_pin, int pwm_pin, int pwm_base);

    void init() const;

    int get_speed() const;

    /**
     * @param new_speed an integer in the range [-100, 100] determining the motor power to drive forward or backward
     */
    void set_speed(int new_speed);

    /**
     * @warning Directly sets the motors pwm pin without updating the objects state
     */
    void set_speed_raw(int new_speed) const;
    void start();
    void stop();

private:
    int get_raw_speed(const int new_speed) const {
        return abs(new_speed) * pwm_base / 100;
    }
    void set_motor_direction(const int new_speed) const {
        if (new_speed > 0) {
            digitalWriteFast(forwards_pin, HIGH);
            digitalWriteFast(backwards_pin, LOW);
        } else if (new_speed < 0){
            digitalWriteFast(forwards_pin, LOW);
            digitalWriteFast(backwards_pin, HIGH);
        } else {
            digitalWriteFast(forwards_pin, LOW);
            digitalWriteFast(backwards_pin, LOW);
        }
    }
};

#endif //MOTOR_HPP
