//
// Created by pepe on 08-08-22.
//

#ifndef PICO_REMOTE_CAR_DC_MOTOR_H
#define PICO_REMOTE_CAR_DC_MOTOR_H


#include "pico/stdlib.h"
#include <cmath>
#include "hardware/pwm.h"
#include "pins.h"

#define TOP 4999

class DCMotor {
public:
    DCMotor(uint enA_pin, uint enB_pin);
    void write_int16(int16_t pwm);
    void write(float duty_cycle);

private:
    uint _enA_pin;
    uint _enB_pin;
    uint _slice_numA;
    uint _slice_numB;
    uint _channelA;
    uint _channelB;
};

#endif //PICO_REMOTE_CAR_DC_MOTOR_H
