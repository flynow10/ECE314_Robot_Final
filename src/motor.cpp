//
// Created by Natalie Wagner on 4/10/26.
//

#include "motor.hpp"

#include <Arduino.h>

#include "digitalWriteFast.h"

Motor::Motor(const int forward_pin, const int backward_pin, const int pwm_pin, const int pwm_base) : forwards_pin(forward_pin), backwards_pin(backward_pin), pwm_pin(pwm_pin), pwm_base(pwm_base), speed(0), moving(false) {
}

void Motor::init() const {
    pinModeFast(forwards_pin, OUTPUT);
    pinModeFast(backwards_pin, OUTPUT);
    pinModeFast(pwm_pin, OUTPUT);
}

int Motor::get_speed() const {
    return speed;
}

void Motor::set_speed(const int new_speed) {
    if (new_speed == 0) {
        stop();
    } else if (moving) {
        if ((this->speed ^ new_speed) < 0) {
            set_motor_direction(new_speed);
        }
        analogWrite(pwm_pin, get_raw_speed(new_speed));
    }
    this->speed = new_speed;
}

void Motor::set_speed_raw(const int new_speed) const {
    analogWrite(pwm_pin, get_raw_speed(new_speed));
}

void Motor::start() {
    if (moving) return;
    set_motor_direction(speed);
    analogWrite(pwm_pin, speed);
    moving = true;
}

void Motor::stop() {
    if (!moving) return;
    set_motor_direction(0);
    moving = false;
}

