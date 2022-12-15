#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include <cmath>

#define I2C_PORT i2c1
#define I2C_SDA_PIN 26
#define I2C_SCL_PIN 27
#define LED_PIN 25

#define READ 'r'
#define COMMAND_POS 'p'
#define COMMAND_VEL 'v'
#define READ_ENCODER 'e'
#define COMMAND_POS_VEL 'a'
#define HOME 'h'

static int SLAVE_ADDR = 0x15;
uint8_t times_led = 0;
uint8_t joint1_sp;

char in_buffer[50];
int input_char_index;
int input_char;
float result[50];

void function_callback(char *buffer, int size_buffer)
{
    char *current;
    char *previous;
    char *token = strtok(buffer, " ");
    char command = *token;

    switch (command)
    {
    case (COMMAND_POS):
        printf("Send position goal was call\n");
        token = strtok(NULL, " ");

        result[0] = strtof(token, &previous);
        printf("%.1f", result[0]);

        for (int i = 0; i < size_buffer - 1; i++)
        {
            result[i + 1] = strtof(previous + 1, &current);
            previous = current;
            printf(", %.1f", result[i + 1]);
        }
        printf("\n");
        break;

    case (COMMAND_VEL):
        printf("Send speed goal was call\n");
        token = strtok(NULL, " ");

        result[0] = strtof(token, &previous);
        printf("%.1f", result[0]);

        for (int i = 0; i < size_buffer - 1; i++)
        {
            result[i + 1] = strtof(previous + 1, &current);
            previous = current;
            printf(", %.1f", result[i + 1]);
        }
        printf("\n");
        break;
    case (READ_ENCODER):

        printf("Encoder callback \n");
        break;

    default:
        printf("Invalid command \n");
    }
}

void waiting_input(int input_std)
{
    while (input_std != PICO_ERROR_TIMEOUT)
    {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        in_buffer[input_char_index++] = input_std;
        if (input_std == '\n')
        {
            in_buffer[input_char_index] = 0;
            input_char_index = 0;
            times_led = uint8_t(strtof(in_buffer, NULL));
            break;
        }
        input_std = getchar_timeout_us(0);
    }
}

void init_i2c_com()
{
    i2c_init(I2C_PORT, 100 * 1000);
    // i2c_set_slave_mode(I2C_PORT, true, SLAVE_ADDR);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);
}

int main()
{
    stdio_init_all();
    init_i2c_com();

    uint8_t read = READ;
    uint8_t result;

    while (true)
    {
        input_char = getchar_timeout_us(0); // Esperar la entrada del usuario

        waiting_input(input_char);

        if (times_led == 6)
        {
            i2c_write_blocking(I2C_PORT, SLAVE_ADDR, &read, 1, true);
            printf("Sending slave msg\n");
            times_led = 0;
        }
        sleep_ms(10);
        // printf("Times LED %d \n", times_led);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
    }
}