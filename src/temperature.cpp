#include "temperature.h"

uint32_t sample_time_ms = 500;

float kp = 0.03324;
float ki = 0.000538; 
float kd = 0.07552;

TEMP_PID::TEMP_PID(){
    PID PID_hotend(&actual_temperture, &output_temperture, &target_temperture, kp, ki, kd, sample_time_ms);
}

void TEMP_PID::temp_init()
{
    _slice_num = pwm_gpio_to_slice_num(CONTROL_PIN);
    _channel = pwm_gpio_to_channel(CONTROL_PIN);
    gpio_set_function(CONTROL_PIN, GPIO_FUNC_PWM);
    pwm_set_wrap(_slice_num, TOP);
    pwm_set_chan_level(_slice_num, _channel, 0);

    pwm_set_enabled(_slice_num, true);

    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(ADC_CHANNEL);

    PID_hotend.set_output_limits(0.0, 1.0);
}

void TEMP_PID::temp_write(float duty_cycle)
{
    if (duty_cycle > 1.0f)
        duty_cycle = 1.0f;
    if (duty_cycle < -1.0f)
        duty_cycle = -1.0f;
    pwm_set_chan_level(_slice_num, _channel, (int16_t)(duty_cycle * TOP));
    pwm_set_enabled(_slice_num, true);
}

void TEMP_PID::temp_calculate()
{
    float temp = -20;

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

    uint16_t raw_read = adc_read();

    for (size_t i = 0; i < 64; i++)
    {
        if (raw_read >= LUT[i][0] && raw_read <= LUT[i + 1][0])
        {
            temp = ((LUT[i + 1][1] - LUT[i][1]) / (LUT[i + 1][0] - LUT[i][0])) * (raw_read - LUT[i + 1][0]) + LUT[i + 1][1];
        }
    }

    actual_temperture = 0.854 * actual_temperture + 0.0728 * temp + 0.0728 * previous_temperature;
    previous_temperature = temp;

    PID_hotend.compute();
    temp_write(output_temperture);
}

int main()
{
    stdio_init_all();
    temp_init();

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    repeating_timer_t timer;
    if (!add_repeating_timer_ms(-sample_time_ms, timerCallback, NULL, &timer))
    {
        printf("Failure by not set timer!! \n");
    }
    target_temperture = 180;

    while (1)
    {
        gpio_put(LED_PIN, 1);
        printf("0, %.3f, %.3f\n", actual_temperture, target_temperture);
        sleep_ms(500);
    }
}