#ifndef PID_TEMPERATURE_H
#define PID_TEMPERATURE_H

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include "hardware/adc.h"
#include "pid_controller.h"

#define CONTROL_PIN 17
#define TOP 4095

#define ADC_PIN 26
#define ADC_CHANNEL 0

class TEMP_PID
{
public:
    TEMP_PID();
    void temp_init();
    void temp_write(float duty_cycle);
    void temp_calculate();
    void set_sample_time(uint32_t new_sample_time_ms);

private:
    float target_temperture, actual_temperture, output_temperture, previous_temperature;
    uint _slice_num;
    uint _channel;
    PID PID_hotend;
};

#endif
