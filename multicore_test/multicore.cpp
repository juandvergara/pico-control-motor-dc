#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"

#define FLAG_VALUE 123
#define LED_PIN 25

void core1_entry()
{

    while (1)
    {
        multicore_fifo_push_blocking(FLAG_VALUE);
        tight_loop_contents();
        uint32_t g = multicore_fifo_pop_blocking();

        if (g != FLAG_VALUE)
            printf("Hmm, that's not right on core 1!\n");
        else
        {
            printf("Its all gone well on core 1!");
            gpio_put(LED_PIN, 1);
            sleep_ms(500);
            gpio_put(LED_PIN, 0);
            sleep_ms(500);
        }
    }
}

int main()
{
    stdio_init_all();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    printf("Hello, multicore!\n");

    multicore_launch_core1(core1_entry);

    // Wait for it to start up

    /*
        if (g != FLAG_VALUE)
            printf("Hmm, that's not right on core 0!\n");
        else
        {
            multicore_fifo_push_blocking(FLAG_VALUE);
            printf("It's all gone well on core 0!");
        }*/
    while (true)
    {
        uint32_t g = multicore_fifo_pop_blocking();
        sleep_ms(100);
        if (g != FLAG_VALUE)
            printf("Hmm, that's not right on core 0!\n");
        else
        {
            multicore_fifo_push_blocking(FLAG_VALUE);
            printf("It's all gone well on core 0!");
        }
    }
}
/*
#define GPIO_ON 1
#define GPIO_OFF 0
#define LED_PIN 25

void second_core_code()
{
    while (1)
    {
        sleep_ms(500);
        multicore_fifo_push_blocking(GPIO_ON);
        sleep_ms(1500);
        multicore_fifo_push_blocking(GPIO_OFF);
    }
}

int main()
{
    multicore_launch_core1(second_core_code);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    while (1)
    {
        uint32_t command = multicore_fifo_pop_blocking();
        gpio_put(LED_PIN, command);
    }
}*/