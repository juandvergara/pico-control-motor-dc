//
// Created by pepe on 14-08-22.
//

#include "encoder.h"

const int QEM[16] = {0, -1, 1, -2, 1, 0, 2, -1, -1, 2, 0, 1, -2, 1, -1, 0};

Encoder::Encoder(uint chA_pin, uint chB_pin)
    : _chA_pin(chA_pin), _chB_pin(chB_pin)
{
    gpio_init(_chA_pin);
    gpio_pull_up(_chA_pin);
    gpio_init(_chB_pin);
    gpio_pull_up(_chB_pin);
}

void Encoder::readPosition(){
    _old_pos = _new_pos;
    _new_pos = gpio_get(_chA_pin) * 2 + gpio_get(_chB_pin) * 1;
    encoder_pos += QEM[_old_pos * 4 + _new_pos];
}