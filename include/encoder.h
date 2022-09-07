//
// Created by pepe on 14-08-22.
//

#ifndef PICO_REMOTE_CAR_ENCODER_H
#define PICO_REMOTE_CAR_ENCODER_H

#include "pico/stdlib.h"
#include "pins.h"

// pins


#define PREV_MASK 0x1
#define CURR_MASK 0x2
#define INVALID_MASK 0x3

extern const uint32_t encoder1_mask; //= (0x01 << M1_ENC_A_PIN) | (0x01 << M1_ENC_B_PIN);
extern const uint32_t encoder2_mask; //= (0x01 << M2_ENC_A_PIN) | (0x01 << M2_ENC_B_PIN);

extern volatile int32_t encoder1_ticks;
extern volatile uint32_t encoder1_state;

extern volatile int32_t encoder2_ticks;
extern volatile uint32_t encoder2_state;

void encoder_setup();
void encoder_callback(uint gpio, uint32_t events);


#endif //PICO_REMOTE_CAR_ENCODER_H
