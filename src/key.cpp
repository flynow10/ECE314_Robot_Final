//
// Created by Natalie Wagner on 4/8/26.
//
// ReSharper disable once CppUnusedIncludeDirective
#include <Wire.h>
#define USE_IRREMOTE_HPP_AS_PLAIN_INCLUDE
#include <IRremote.hpp>
#include "key.hpp"

struct Key keys[] = {
    {.name = "CH-", .command = 0x45},
    {.name = "CH", .command = 0x46},
    {.name = "CH+", .command = 0x47},
    {.name = "Prev", .command = 0x44},
    {.name = "Next", .command = 0x40},
    {.name = "Play", .command = 0x43},
    {.name = "-", .command = 0x7},
    {.name = "+", .command = 0x15},
    {.name = "EQ", .command = 0x9},
    {.name = "0", .command = 0x16},
    {.name = "100+", .command = 0x19},
    {.name = "200+", .command = 0xd},
    {.name = "1", .command = 0xc},
    {.name = "2", .command = 0x18},
    {.name = "3", .command = 0x5e},
    {.name = "4", .command = 0x8},
    {.name = "5", .command = 0x1c},
    {.name = "6", .command = 0x5a},
    {.name = "7", .command = 0x42},
    {.name = "8", .command = 0x52},
    {.name = "9", .command = 0x4a},
  };

const Key& findKey(const int command) {
    for(auto& key : keys) {
        if(key.command == command) {
            return key;
        }
    }

    return NULL_KEY;
}

const Key& waitForKeyPress() {
    // Serial.println("Waiting");
    while(!IrReceiver.decode()) {
        delay(10);
    }
    auto command = IrReceiver.decodedIRData.command;
    auto flags = IrReceiver.decodedIRData.flags;
    auto protocol = IrReceiver.decodedIRData.protocol;
    IrReceiver.resume();
    if(protocol == UNKNOWN || flags & IRDATA_FLAGS_IS_REPEAT) {
        return waitForKeyPress();
    }
    // IrReceiver.printIRResultAsCVariables(&Serial);
    // Serial.println("Read key: " + String(command, 16));
    return findKey(command);
}

const Key* tryReadKey() {
    if(!IrReceiver.decode()) {
        return nullptr;
    }
    auto command = IrReceiver.decodedIRData.command;
    auto flags = IrReceiver.decodedIRData.flags;
    auto protocol = IrReceiver.decodedIRData.protocol;
    IrReceiver.resume();
    if(protocol == UNKNOWN || flags & IRDATA_FLAGS_IS_REPEAT) {
        return nullptr;
    }
    return &findKey(command);
}