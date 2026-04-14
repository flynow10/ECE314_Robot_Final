//
// Created by Natalie Wagner on 4/8/26.
//

#ifndef KEY_HPP
#define KEY_HPP
#ifndef KEY_H
#define KEY_H
struct Key {
    String name;
    int command;
};

const Key NULL_KEY = { .name="Unknown", .command = 0 };

const Key& findKey(const int command);
const Key& waitForKeyPress();
const Key* tryReadKey();

#endif
#endif //KEY_HPP
