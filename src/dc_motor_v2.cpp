//
// Created by pepe on 08-08-22.
//

#include "dc_motor_v2.h"

DCMotor::DCMotor(uint enA_pin, uint enB_pin)
    : _enA_pin(enA_pin), _enB_pin(enB_pin)
{
    // init PWM
    _slice_numA = pwm_gpio_to_slice_num(_enA_pin);
    _slice_numB = pwm_gpio_to_slice_num(_enB_pin);
    _channelA = pwm_gpio_to_channel(_enA_pin);
    _channelB = pwm_gpio_to_channel(_enB_pin);
    gpio_set_function(_enA_pin, GPIO_FUNC_PWM);
    gpio_set_function(_enB_pin, GPIO_FUNC_PWM);
    pwm_set_wrap(_slice_numA, TOP);
    pwm_set_wrap(_slice_numB, TOP);
    pwm_set_chan_level(_slice_numA, _channelA, 0);
    pwm_set_chan_level(_slice_numB, _channelB, 0);
    pwm_set_output_polarity(_slice_numA, true, false); // Channel A inverted B normal mode

    //
    pwm_set_enabled(_slice_numA, true);
    pwm_set_enabled(_slice_numB, true);
}

void DCMotor::write_int16(int16_t pwm)
{
    /*if(pwm < 0)
    {
        pwm_set_chan_level(_slice_numA, _channelA, abs(pwm));
        pwm_set_chan_level(_slice_numB, _channelB, abs(pwm));
    } else
    {*/
    pwm_set_chan_level(_slice_numA, _channelA, pwm);
    pwm_set_chan_level(_slice_numB, _channelB, pwm);
    //}
    pwm_set_enabled(_slice_numA, true);
    pwm_set_enabled(_slice_numB, true);
}

void DCMotor::write(float duty_cycle)
{
    if (duty_cycle > 1.0f)
        duty_cycle = 1.0f;
    if (duty_cycle < -1.0f)
        duty_cycle = -1.0f;
    duty_cycle = duty_cycle * 0.5f + 0.5f;
    write_int16((int16_t)(duty_cycle * TOP));
}