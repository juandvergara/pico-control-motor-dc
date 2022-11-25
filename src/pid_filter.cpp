#include "pid_filter.h"

PID::PID(float *input, float *dot_input, float *output, float *setpoint, float *dot_setpoint,
         float kp, float ki, float kd, uint32_t sample_time_ms)
    : _my_input(input), _my_dot_input(dot_input), _my_output(output), _my_setpoint(setpoint), _my_dot_setpoint(dot_setpoint),
      _sample_time_ms(sample_time_ms), _out_sum(0)
{
    set_output_limits(-1.0f, 1.0f);
    set_gains(kp, ki, kd);
    this->_last_err = 0.0;
    this->_last_output = 0.0;
}

void PID::set_sample_time(uint32_t new_sample_time_ms)
{
    float ratio = (float)new_sample_time_ms / _sample_time_ms;

    _ki *= ratio;
    //_kd /= ratio;
    _sample_time_ms = new_sample_time_ms;
}

void PID::set_gains(float kp, float ki, float kd)
{
    if (kp < 0 || ki < 0 || kd < 0)
        return;

    float sample_time_s = (float)_sample_time_ms / 1000.0f;
    printf("pid sample time: %f [s]", sample_time_s);

    _kp = kp;
    //_ki = ki * sample_time_s;
    _ki = ki / sample_time_s; // Cause discrete PI works whith this
    _kd = kd;
}

void PID::set_output_limits(float min, float max)
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

void PID::compute(void)
{
    float input = *_my_input;
    float error = *_my_setpoint - input;
    float dot_input = *_my_dot_input;
    float dot_error = *_my_dot_setpoint - dot_input;

    float output = _last_output + error * (_kp + _ki) - _last_err * _kp + dot_error * _kd;

    //float output = _last_output + (_kp + _ki + _kd) * error - (_kp + 2.0 * _kd) * _last_err + _kd * _last_err2;

    if (output > _out_max)
        output = _out_max;
    else if (output < _out_min)
        output = _out_min;

    *_my_output = output;
    _last_output = output;
    _last_err = error;
}
