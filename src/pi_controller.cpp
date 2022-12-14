//
// Created by pepe on 21/6/21.
//

#include "pi_controller.h"

PI::PI(float *input, float *output, float *setpoint, float kp, float ki, uint32_t sample_time_ms)
    : _my_input(input), _my_output(output), _my_setpoint(setpoint), _sample_time_ms(sample_time_ms), _out_sum(0)
{
    set_output_limits(-1.0f, 1.0f);
    set_gains(kp, ki);
    this->_last_err = 0.0;
    this->_last_output = 0.0;
}

void PI::set_sample_time(uint32_t new_sample_time_ms)
{
    float ratio = (float)new_sample_time_ms / _sample_time_ms;

    _ki /= ratio;
    _sample_time_ms = new_sample_time_ms;
}

void PI::set_gains(float kp, float ki)
{
    if (kp < 0 || ki < 0)
        return;

    float sample_time_s = (float)_sample_time_ms / 1000.0f;
    printf("pid sample time: %f [s]", sample_time_s);

    _kp = kp;
    _ki = ki / _sample_time_ms; // Se cambió por un intento de discretizado
}

void PI::set_output_limits(float min, float max)
{
    if (min >= max)
        return;
    _out_min = min;
    _out_max = max;

    if (*_my_output > _out_max)
        *_my_output = _out_max;
    else if (*_my_output < _out_min)
        *_my_output = _out_min;

    if (_out_sum > _out_max)
        _out_sum = _out_max;
    else if (_out_sum < _out_min)
        _out_sum = _out_min;
}

void PI::compute(void)
{
    float input = *_my_input;
    float error = *_my_setpoint - input;

    float output = _last_output + error * (_kp + _ki) - _last_err * _kp;

    if (output > _out_max)
        output = _out_max;
    else if (output < _out_min)
        output = _out_min;

    *_my_output = output;
    _last_output = output;
    _last_err = error;
}
