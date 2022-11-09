#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include <cmath>

#define I2C_PORT i2c1
#define I2C_SDA_PIN 26
#define I2C_SCL_PIN 27
#define LED_PIN 25

static int SLAVE_ADDR = 0x15;

int main()
{
    stdio_init_all();

    i2c_init(I2C_PORT, 100 * 1000);
    i2c_set_slave_mode(I2C_PORT, true, SLAVE_ADDR);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);

    uint8_t times_led = 0;
    int input_char;
    int input_char_index;
    char *char_pt1;
    char in_buffer[100];

    uint8_t joint1_sp;

    while (true)
    {
        input_char = getchar_timeout_us(0); // Esperar la entrada del usuario

        while (input_char != PICO_ERROR_TIMEOUT)
        {
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            putchar(input_char);
            in_buffer[input_char_index++] = input_char;
            if (input_char == '/')
            {
                in_buffer[input_char_index] = 0;
                input_char_index = 0;
                joint1_sp = strtof(in_buffer, &char_pt1);
                break;
            }
            times_led = uint8_t(joint1_sp);
            input_char = getchar_timeout_us(0);
            printf("\n Caracter recibido \n");
        }
        sleep_ms(500);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        i2c_write_raw_blocking(I2C_PORT, &times_led, 1);
        sleep_ms(5000);
        printf("Valor de veces LED %d \n", times_led);
    }
}