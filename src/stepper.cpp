#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#define STEP_PIN 15
#define DIR_PIN 14

const uint STEPS_PER_REVOLUTION = 200;
const uint MICROSTEPS = 32;
const uint MS_DELAY = 1;

void step(bool dir)
{
    gpio_put(DIR_PIN, dir);
    gpio_put(STEP_PIN, 1);
    sleep_us(1);
    gpio_put(STEP_PIN, 0);
    sleep_ms(MS_DELAY);
}

void rotateDegrees(int degrees, int speed)
{
    int steps = degrees * (STEPS_PER_REVOLUTION / 360.0) * MICROSTEPS;
    float delay = 1.0 / speed;
    for (int i = 0; i < steps; i++)
    {
        step(1);
        sleep_us(delay * 1000000);
    }
}

void init_stepper()
{
    gpio_init(STEP_PIN);
    gpio_init(DIR_PIN);
    gpio_set_dir(STEP_PIN, GPIO_OUT);
    gpio_set_dir(DIR_PIN, GPIO_OUT);
}