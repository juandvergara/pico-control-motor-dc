#include "pico/stdlib.h"
#include <cmath>
#include "dc_motor_v2.h"

DCMotor motor1(M1_ENA_PIN, M1_ENB_PIN);
DCMotor motor2(M2_ENA_PIN, M2_ENB_PIN);
DCMotor motor3(M3_ENA_PIN, M3_ENB_PIN);

uint32_t millis() {
    return to_ms_since_boot(get_absolute_time());
}

int main() {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    while (1) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        motor1.write(0.0);
        motor2.write(0.5);
        motor3.write(-0.5);
    }
}
