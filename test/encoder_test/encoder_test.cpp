#include "pico/stdlib.h"
#include <cstdio>
#include <cmath>
//#include "encoder_cuadratura.h"
#include "encoder.h"

Encoder encoder_m1(M1_ENC_A_PIN, M1_ENC_B_PIN);
Encoder encoder_m2(M2_ENC_A_PIN, M2_ENC_B_PIN);
Encoder encoder_m3(M3_ENC_A_PIN, M3_ENC_B_PIN);

void encoders_callback(uint gpio, uint32_t events)
{
    encoder_m1.readPosition();
    encoder_m2.readPosition();
    encoder_m3.readPosition();
}

uint32_t millis()
{
    return to_ms_since_boot(get_absolute_time());
}

int main()
{
    stdio_init_all();
    printf("Encoder Test");
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    gpio_set_irq_enabled_with_callback(M1_ENC_A_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoders_callback);
    gpio_set_irq_enabled_with_callback(M2_ENC_A_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoders_callback);
    gpio_set_irq_enabled_with_callback(M3_ENC_A_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoders_callback);

    while (1)
    {
        printf("M1: %f, M2: %f, M3: %f\n", float(encoder_m1.encoder_pos), float(encoder_m2.encoder_pos), float(encoder_m3.encoder_pos));
        sleep_ms(100);
    }
}