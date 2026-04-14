//
// Created by Natalie Wagner on 4/8/26.
//

#ifndef PID_HPP
#define PID_HPP

#define INTEGRAL_WIDTH 10

template <typename T = int> class PID {
public:
    PID(T K_p, T K_i, T K_d);
    ~PID() = default;
    T p_error = 0;
    T i_error = 0;
    T d_error = 0;

    T K_p;
    T K_i;
    T K_d;

    void update_error(T new_error);
    void flush();
    T get_output();

private:
    T i_cumulative_error[INTEGRAL_WIDTH] = {};
    int i_index = 0;
};

#endif // PID_HPP
