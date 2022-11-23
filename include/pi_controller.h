//
// Created by pepe on 21/6/21.
//

#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H

#include "pico/stdlib.h"
#include "dc_motor.h"
#include "encoder_cuadratura.h"
#include "cstdio"

class PI
{
public:
    PID(float *input, float *output, float *setpoint, float kp, float ki, uint32_t sample_time_ms);
    void compute(void);
    void set_output_limits(float min, float max);
    void set_gains(float kp, float ki);
    void set_sample_time(uint32_t new_sample_time_ms);

private:
    uint32_t _sample_time_ms;
    uint32_t _last_time;
    float _kp;
    float _ki;

    float *_my_input;
    float *_my_output;
    float *_my_setpoint;
    float _out_min;
    float _out_max;
    float _out_sum;
    float _err_sum;
    float _last_err;
    float _last_output;
};

#endif PI_CONTROLLER_H
