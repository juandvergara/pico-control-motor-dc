//
// Created by pepe on 14-08-22.
//

#ifndef PICO_REMOTE_CAR_ENCODER_H
#define PICO_REMOTE_CAR_ENCODER_H

#include "pico/stdlib.h"
#include "pins.h"

// pins

extern unsigned char Old_Pos, New_Pos;
extern volatile long encoder0PosM1;
extern volatile float PosEncoderM1;
extern const int QEM [16];

void encoder_setup();
void encoder_callback(uint gpio, uint32_t events);

#endif //PICO_REMOTE_CAR_ENCODER_H
