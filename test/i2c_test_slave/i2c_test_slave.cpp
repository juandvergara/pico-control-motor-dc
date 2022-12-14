#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"

#define I2C_PORT i2c0
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5
#define LED_PIN 25

#define READ 'r'

uint8_t command;

static int SLAVE_ADDR = 0x15;

void init_slave()
{
    i2c_init(I2C_PORT, 100 * 1000);
    i2c_set_slave_mode(I2C_PORT, true, SLAVE_ADDR);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
}

int main()
{
    stdio_init_all();
    init_slave(); 
    uint8_t times_led = 0;
    uint8_t ihave = '6';
    sleep_ms(3000);

    while (true)
    {
        i2c_read_raw_blocking(I2C_PORT, &command, 1);
        if (command == READ)
        {
            printf("Here im master\n");
            command = ' ';
        }
    }
}

// for (uint8_t i = 0; i < times_led; i++)
// {
//     gpio_put(LED_PIN, 1);
//     sleep_ms(250);
//     gpio_put(LED_PIN, 0);
//     sleep_ms(250);
// }
// sleep_ms(100);