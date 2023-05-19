#include "pid_v3.h"

PID_V3::PID_V3(float *input, float *dot_input, float *output, float *setpoint, float *dot_setpoint,
               float kp, float ki, float kd, uint32_t sample_time_ms, float k_h, float k_gamma)
    : _my_input(input), _my_dot_input(dot_input), _my_output(output), _my_setpoint(setpoint), _my_dot_setpoint(dot_setpoint),
      _sample_time_ms(sample_time_ms), _out_sum(0)
{
    set_output_limits(-1.0f, 1.0f);
    set_gains(kp, ki, kd, k_h, k_gamma);
    this->_last_err = 0.0;
    this->_last_output = 0.0;
    this->_last_integrator = 0.0;
}

void PID_V3::set_sample_time(uint32_t new_sample_time_ms)
{
    float ratio = (float)new_sample_time_ms / _sample_time_ms;

    _ki *= ratio;
    //_kd /= ratio;
    _sample_time_ms = new_sample_time_ms;
}

void PID_V3::set_gains(float kp, float ki, float kd, float k_h, float k_gamma)
{
    if (kp < 0 || ki < 0 || kd < 0)
        return;

    float sample_time_s = (float)_sample_time_ms / 1000.0f;
    printf("pid set p: %f, i: %f, d: %f. Rate: %f [s] \n", kp, ki, kd, sample_time_s);

    _kp = kp;
    _ki = ki * sample_time_s; // Cause discrete PI works whith this
    _kd = kd;
    _k_h = k_h;
    _k_gamma = k_gamma;
}

void PID_V3::set_output_limits(float min, float max)
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

void PID_V3::compute(void)
{
    float input = *_my_input;
    float error = *_my_setpoint - input;
    float dot_input = *_my_dot_input;
    float dot_error = *_my_dot_setpoint - dot_input;
    _last_integrator = _last_integrator + (error + _last_err) / 2.0;

    float output = _kd * dot_error + _kp * error + _ki * _last_integrator;
    output = output * (_k_h + 1 / pow(_k_gamma, 2));
    // float output = _kp * error + _kd * (error - _last_err);

    if (output > _out_max)
        output = _out_max;
    else if (output < _out_min)
        output = _out_min;

    *_my_output = output;
    _last_output = output;
    _last_err = error;
}
