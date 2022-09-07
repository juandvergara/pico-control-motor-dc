#include <cstdio>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "encoder_cuadratura.h"

unsigned char Old_Pos, New_Pos;
volatile long encoder0PosM1 = 0;
volatile float PosEncoderM1 = 0;
const int QEM[16] = {0, -1, 1, -2, 1, 0, 2, -1, -1, 2, 0, 1, -2, 1, -1, 0};

void encoder_callback(uint gpio, uint32_t events)
{
    Old_Pos = New_Pos;
    New_Pos = gpio_get(M1_ENC_A_PIN) * 2 + gpio_get(M1_ENC_B_PIN) * 1;
    encoder0PosM1 = QEM[Old_Pos * 4 + New_Pos];
    PosEncoderM1 += encoder0PosM1;
    printf("[%f]\n", PosEncoderM1*360.0/(80.0*127.7*4)); // Dientes de salida motor 18, plato receptor 72, relación 4:1
}

void encoder_setup()
{
    gpio_init(M1_ENC_A_PIN);
    gpio_pull_up(M1_ENC_A_PIN);
    gpio_init(M1_ENC_B_PIN);
    gpio_pull_up(M1_ENC_B_PIN);
    /*  gpio_init(M2_ENC_A_PIN);
        gpio_pull_up(M2_ENC_A_PIN);
        gpio_init(M2_ENC_B_PIN);
        gpio_pull_up(M2_ENC_B_PIN);
    */
    gpio_set_irq_enabled_with_callback(M1_ENC_A_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_callback);
    // gpio_set_irq_enabled_with_callback(M2_ENC_A_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_callback);
}
