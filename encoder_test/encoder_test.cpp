#include "pico/stdlib.h"
#include <cstdio>
#include <cmath>
#include "encoder_cuadratura.h"

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

    encoder_setup();

    while (1)
    {
       // printf("[%d], \t[%d]\n", encoder1_ticks, encoder2_ticks);
        // printf("[%d]\n", New_Pos);
        sleep_ms(100);
    }
}