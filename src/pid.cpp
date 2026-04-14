//
// Created by Natalie Wagner on 4/8/26.
//

#include "pid.hpp"

template <typename T>
PID<T>::PID(T K_p, T K_i, T K_d) : K_p(K_p), K_i(K_i), K_d(K_d) {}

template <typename T> void PID<T>::update_error(T new_error) {
    d_error = new_error - p_error;
    p_error = new_error;

    i_index = (i_index + 1) % INTEGRAL_WIDTH;

    i_error += new_error - i_cumulative_error[i_index];
    i_cumulative_error[i_index] = new_error;
}

template <typename T> void PID<T>::flush() {
    memset(i_cumulative_error, 0, sizeof(i_cumulative_error));
    i_index = 0;
}

template <typename T> T PID<T>::get_output() {
    T output = 0;
    output += K_p * p_error;
    output += K_i * i_error;
    output += K_d * d_error;
    return output;
}
