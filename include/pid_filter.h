//
// Created by pepe on 21/6/21.
//

#ifndef PID_V2_CONTROLLER_H
#define PID_V2_CONTROLLER_H

#include "pico/stdlib.h"
#include "cstdio"

class PID_V2
{
public:
    PID_V2(float *input, float *dot_input, float *output, float *setpoint, float *dot_setpoint,
        float kp, float ki, float kd, uint32_t sample_time_ms);
    void compute(void);
    void set_output_limits(float min, float max);
    void set_gains(float kp, float ki, float kd);
    void set_sample_time(uint32_t new_sample_time_ms);

private:
    uint32_t _sample_time_ms;
    uint32_t _last_time;
    float _kp;
    float _kd;
    float _ki;

    float *_my_input;
    float *_my_dot_input;
    float *_my_output;
    float *_my_setpoint;
    float *_my_dot_setpoint;
    float _out_min;
    float _out_max;
    float _out_sum;
    float _err_sum;
    float _last_err;
    float _last_input;
    float _last_output;
};

#endif // DIFF_DRIVE_PID_CONTROLLER_H
