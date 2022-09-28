//
// Created by pepe on 14-08-22.
//

#ifndef PICO_REMOTE_CAR_ENCODER_H
#define PICO_REMOTE_CAR_ENCODER_H

#include "pico/stdlib.h"
#include "pins.h"

// pins
class Encoder
{
private:
    uint _chA_pin;
    uint _chB_pin;
    unsigned char _old_pos, _new_pos;
    
public:
    volatile long encoder_pos;
    Encoder(uint chA_pin, uint chB_pin);
    void readPosition();
};

#endif // PICO_REMOTE_CAR_ENCODER_H
