//
// Created by Natalie Wagner on 4/10/26.
//

#include "ultrasonic.hpp"

#include "digitalWriteFast.h"
#include "pins.hpp"

ulong microsecondsToInches(const ulong microseconds) {
    return microseconds / 74 / 2;
}

ulong readUltrasonic() {
    digitalWriteFast(UltrasonicTriggerPin, LOW);
    delayMicroseconds(2);
    digitalWriteFast(UltrasonicTriggerPin, HIGH);
    delayMicroseconds(10);
    digitalWriteFast(UltrasonicTriggerPin, LOW);
    return pulseIn(UltrasonicEchoPin, HIGH);
}

void initUltrasonic() {
    pinMode(UltrasonicTriggerPin, OUTPUT);
    pinMode(UltrasonicEchoPin, INPUT);
}
