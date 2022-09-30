#include "pico/stdlib.h"
#include <cstdio>
#include <cstring>
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "pico/util/queue.h"

int main()
{
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    while (true)
    {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(500);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(500);
    }
}
