#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"

#define I2C_PORT i2c0
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5
#define LED_PIN 25

static int SLAVE_ADDR = 0x15;

int main()
{
    stdio_init_all();

    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);

    uint8_t times_led = 0;

    while (true)
    {
        times_led++;
        if (times_led == 10)
        {
            times_led = 0;
        }
        i2c_write_blocking(I2C_PORT, SLAVE_ADDR, &times_led, 1, false);
        sleep_ms(10000);
    }
}