#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"

#define _pwm_pin 15
#define TOP 4095
uint _slice_num;
uint _channel;

float temp_calculate(uint16_t raw_read)
{
    float temp = 25;

    const float LUT[][2] = {
        {92, 300},
        {100, 295},
        {108, 290},
        {112, 285},
        {124, 280},
        {132, 275},
        {140, 270},
        {152, 265},
        {164, 260},
        {176, 255},
        {192, 250},
        {208, 245},
        {224, 240},
        {244, 235},
        {264, 230},
        {284, 225},
        {312, 220},
        {336, 215},
        {368, 210},
        {400, 205},
        {436, 200},
        {480, 195},
        {524, 190},
        {572, 185},
        {624, 180},
        {685, 175},
        {749, 170},
        {821, 165},
        {897, 160},
        {981, 155},
        {1073, 150},
        {1173, 145},
        {1281, 140},
        {1393, 135},
        {1517, 130},
        {1645, 125},
        {1781, 120},
        {1921, 115},
        {2066, 110},
        {2214, 105},
        {2366, 100},
        {2514, 95},
        {2662, 90},
        {2810, 85},
        {2950, 80},
        {3082, 75},
        {3206, 70},
        {3322, 65},
        {3431, 60},
        {3527, 55},
        {3615, 50},
        {3691, 45},
        {3759, 40},
        {3819, 35},
        {3867, 30},
        {3911, 25},
        {3943, 20},
        {3975, 15},
        {3999, 10},
        {4019, 5},
        {4035, 0},
        {4051, -5},
        {4067, -10},
        {4083, -15}};

    for (size_t i = 0; i < 64; i++)
    {
        if (raw_read >= LUT[i][0] && raw_read <= LUT[i + 1][0])
        {
            temp = ((LUT[i + 1][1] - LUT[i][1]) / (LUT[i + 1][0] - LUT[i][0])) * (raw_read - LUT[i + 1][0]) + LUT[i + 1][1];
        }
    }
    return temp;
}

void pwm_temp_init()
{
    _slice_num = pwm_gpio_to_slice_num(_pwm_pin);
    _channel = pwm_gpio_to_channel(_pwm_pin);
    gpio_set_function(_pwm_pin, GPIO_FUNC_PWM);
    pwm_set_wrap(_slice_num, TOP);
    pwm_set_chan_level(_slice_num, _channel, 0);

    pwm_set_enabled(_slice_num, true);
}

void temp_write_int16(int16_t pwm)
{
    pwm_set_chan_level(_slice_num, _channel, pwm);
    pwm_set_enabled(_slice_num, true);
}

void temp_write(float duty_cycle)
{
    if (duty_cycle > 1.0f)
        duty_cycle = 1.0f;
    if (duty_cycle < -1.0f)
        duty_cycle = -1.0f;
    temp_write_int16((int16_t)(duty_cycle * TOP));
}

int main()
{
    stdio_init_all();

    adc_init();

    pwm_temp_init();

    // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(26);
    // Select ADC input 0 (GPIO26)
    adc_select_input(0);

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    while (1)
    {
        // 12-bit conversion, assume max value == ADC_VREF == 3.3 V
        // const float conversion_factor = 3.3f / (1 << 12);
        uint16_t result = adc_read();
        float temp_res = temp_calculate(result);

        printf("Temperature: %.3f °C \n", temp_res);
        if (temp_res < 180.0)
        {
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            printf("Calentando... \n");
            temp_write(0.5);
        }

        sleep_ms(1000);
    }
}