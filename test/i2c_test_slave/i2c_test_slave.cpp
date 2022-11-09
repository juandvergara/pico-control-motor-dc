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
    //i2c_set_slave_mode(I2C_PORT, true, SLAVE_ADDR);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    uint8_t times_led = 0;
    sleep_ms(3000);

    while (true)
    {
        //printf("Antes de leer %d \n", i2c_get_read_available(I2C_PORT));
        //i2c_get_read_available(I2C_PORT);
        //i2c_read_blocking_until(I2C_PORT, SLAVE_ADDR, &times_led, 1, false, 200);
        i2c_read_blocking(I2C_PORT, SLAVE_ADDR, &times_led, 1, false);
        printf("despues de leer %d \n", times_led);
        for (uint8_t i = 0; i < times_led; i++)
        {
            gpio_put(LED_PIN, 1);
            sleep_ms(250);
            gpio_put(LED_PIN, 0);
            sleep_ms(250);
        }
    }
}