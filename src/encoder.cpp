//
// Created by pepe on 14-08-22.
//

#include "encoder.h"

const uint32_t encoder1_mask = (0x01 << M1_ENC_A_PIN) | (0x01 << M1_ENC_B_PIN);
const uint32_t encoder2_mask = (0x01 << M2_ENC_A_PIN) | (0x01 << M2_ENC_B_PIN);

volatile int32_t encoder1_ticks;
volatile int32_t encoder2_ticks;

volatile uint32_t encoder1_state;
volatile uint32_t encoder2_state;


void encoder_callback(uint gpio, uint32_t events) {
    int32_t change1 = 0;
    int32_t change2 = 0;
    uint32_t new_state1 = ((gpio_get_all() & encoder1_mask) >> (M1_ENC_B_PIN > M1_ENC_A_PIN ? M1_ENC_A_PIN : M1_ENC_B_PIN)) & 0x3;
    uint32_t new_state2 = ((gpio_get_all() & encoder2_mask) >> (M2_ENC_B_PIN > M2_ENC_A_PIN ? M2_ENC_A_PIN : M2_ENC_B_PIN)) & 0x3;

    if(((new_state1 ^ encoder1_state) != INVALID_MASK) && (new_state1 != encoder1_state))
    {
        change1 = (encoder1_state & PREV_MASK) ^ ((new_state1 & CURR_MASK) >> 1);
        if(change1 == 0)
        {
            change1 = -1;
        }
        if(M1_ENC_INVERTED) change1 = -1 * change1;
        encoder1_ticks -= change1;
    }

    if(((new_state2 ^ encoder2_state) != INVALID_MASK) && (new_state2 != encoder2_state))
    {
        change2 = (encoder2_state & PREV_MASK) ^ ((new_state2 & CURR_MASK) >> 1);
        if(change2 == 0)
        {
            change2 = -1;
        }
        if(M2_ENC_INVERTED) change2 = -1 * change2;
        encoder2_ticks -= change2;
    }
    encoder1_state = new_state1;
    encoder2_state = new_state2;
}

void encoder_setup() {
    gpio_init(M1_ENC_A_PIN);
    gpio_pull_up(M1_ENC_A_PIN);
    gpio_init(M1_ENC_B_PIN);
    gpio_pull_up(M1_ENC_B_PIN);
    gpio_init(M2_ENC_A_PIN);
    gpio_pull_up(M2_ENC_A_PIN);
    gpio_init(M2_ENC_B_PIN);
    gpio_pull_up(M2_ENC_B_PIN);

    gpio_set_irq_enabled_with_callback(M1_ENC_A_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_callback);
    gpio_set_irq_enabled_with_callback(M2_ENC_A_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_callback);

    encoder1_ticks = 0;
    encoder2_ticks = 0;

    encoder1_state = ((gpio_get_all() & encoder1_mask) >> M1_ENC_A_PIN) & 0x3;
    encoder2_state = ((gpio_get_all() & encoder2_mask) >> M2_ENC_A_PIN) & 0x3;
}
