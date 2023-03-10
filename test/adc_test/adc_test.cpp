#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"

// Define los pines para el driver DRV8825
const uint LED_PIN = 25; // Led interno para indicar que el programa está ejecutando
const uint STEP_PIN = 15; // Pin STEP del driver DRV8825
const uint DIR_PIN = 14; // Pin DIR del driver DRV8825

// Configuración de los pines para el motor paso a paso
const uint STEPS_PER_REVOLUTION = 200; // Número de pasos para completar una revolución del motor
const uint MICROSTEPS = 32; // Número de microsteps por paso
const uint MS_DELAY = 1; // Retardo en milisegundos entre cada microstep

void step(bool dir) {
    gpio_put(DIR_PIN, dir);
    gpio_put(STEP_PIN, 1);
    sleep_us(1);
    gpio_put(STEP_PIN, 0);
    sleep_ms(MS_DELAY);
}

void rotateDegrees(int degrees, int speed) {
    int steps = degrees * (STEPS_PER_REVOLUTION / 360.0) * MICROSTEPS;
    float delay = 1.0 / speed;
    for (int i = 0; i < steps; i++) {
        step(1);
        sleep_us(delay * 1000000);
    }
}

int main() {
    stdio_init_all();

    // Configura los pines como salidas
    gpio_init(LED_PIN);
    gpio_init(STEP_PIN);
    gpio_init(DIR_PIN);

    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_set_dir(STEP_PIN, GPIO_OUT);
    gpio_set_dir(DIR_PIN, GPIO_OUT);

    // Configura el PWM para el led interno
    pwm_config config = pwm_get_default_config();
    pwm_set_wrap(pwm_gpio_to_slice_num(LED_PIN), 255);
    pwm_init(pwm_gpio_to_slice_num(LED_PIN), &config, false);
    pwm_set_enabled(pwm_gpio_to_slice_num(LED_PIN), true);

    // Indica que el programa está ejecutándose con el led interno
    gpio_put(LED_PIN, 1);

    // Gira el motor paso a paso 90 grados a una velocidad de 100 revoluciones por minuto
    //rotateDegrees(90, 100);

    // Detiene el motor
    gpio_put(DIR_PIN, 0);
    for (int i = 0; i < 100000; i++) {
        gpio_put(STEP_PIN, 1);
        sleep_us(100);
        gpio_put(STEP_PIN, 0);
        sleep_us(100);
    }

    // Indica que el programa ha finalizado con el led interno
    gpio_put(LED_PIN, 0);

    return 0;
}
