#include "pico/stdlib.h"
#include <cmath>
#include <stdio.h>
#include "dc_motor_v2.h"
#include "encoder.h"
#include "pi_controller.h"

#define PI 3.14159265359f

Encoder encoder3(M3_ENC_A_PIN, M3_ENC_B_PIN);

DCMotor motor3(M3_ENA_PIN, M3_ENB_PIN);

float kp1 = 0.5;
float kd1 = 1.0;
float ki1 = 0.125;

float joint_input1, joint_effort1, joint_setpoint1 = 0.0;
float joint_position1, past_joint_position1;
float joint_velocity1;
float speed_target = 0;

float v1Filt = 0;
float v1Prev = 0;

uint32_t sample_time_ms = 10;
float pid_rate;

char in_buffer[500];
uint16_t char_idx = 0;

float theta = 1.0 / (2.0 * PI * 10.0);

PID PID_Joint1(&joint_input1, &joint_effort1, &joint_setpoint1, kp1, ki1, kd1, sample_time_ms);

uint32_t millis()
{
    return to_ms_since_boot(get_absolute_time());
}

void initRobot()
{
    motor3.write(0.0);
    PID_Joint1.set_output_limits(-1.0f, 1.0f);
    joint_setpoint1 = 0;
    pid_rate = float(sample_time_ms) / 1000.0f;
}

void updatePid(int32_t joint1_encoder_ticks)
{
    int32_t joint1_ticks = joint1_encoder_ticks;

    float motor1_vel = 0;
    past_joint_position1 = joint_position1;
    joint_position1 = float(joint1_ticks) * 360.0f / (80.0f * 127.7f * 4.0f);
    joint_input1 = joint_position1;

    joint_velocity1 = (joint_position1 - past_joint_position1) / pid_rate;
    v1Filt = 0.854 * v1Filt + 0.0728 * joint_velocity1 + 0.0728 * v1Prev;
    v1Prev = joint_velocity1;

    PID_Joint1.compute();

    motor1_vel = joint_effort1 + (speed_target - v1Filt) * 0.15;

    motor3.write(-motor1_vel);
}

bool timerCallback(repeating_timer_t *rt)
{
    updatePid(int32_t(encoder3.encoder_pos));
    return true;
}

void encoders_callback(uint gpio, uint32_t events)
{
    encoder3.readPosition();
}

int main()
{
    stdio_init_all();
    printf("PID Motor test");
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    initRobot();

    gpio_set_irq_enabled_with_callback(M3_ENC_A_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoders_callback);

    repeating_timer_t timer;
    if (!add_repeating_timer_ms(-sample_time_ms, timerCallback, NULL, &timer))
    {
        printf("Failure by not set timer!! \n");
    }

    float theta0 = 0;
    float dot_theta0 = 0;
    float thetaf = 90;
    float dot_thetaf = 0;
    float t_final = 3;

    float a = theta0;
    float b = dot_theta0;
    float c = 3.0 * (thetaf - theta0) / pow(t_final, 2) - 2 * dot_theta0 / pow(t_final, 2) - dot_thetaf / pow(t_final, 2);
    float d = -2.0 * (thetaf - theta0) / pow(t_final, 3) + (dot_thetaf - dot_theta0) / pow(t_final, 2);

    int input_char;
    int input_char_index;
    char *char_pt1;

    float joint1_sp = 0.0;

    sleep_ms(3000);

    while (true)
    {
        input_char = getchar_timeout_us(0);
        while (input_char != PICO_ERROR_TIMEOUT)
        {
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            putchar(input_char);                        // Print user input in console
            in_buffer[input_char_index++] = input_char; // Index user input to buffer array
            if (input_char == '/')
            {
                in_buffer[input_char_index] = 0;
                input_char_index = 0;
                joint1_sp = strtof(in_buffer, &char_pt1);
                break;
            }
            input_char = getchar_timeout_us(0);
            joint_setpoint1 = joint1_sp;
        }
        float currT = millis() / 1e3 - 3;
        if (currT < t_final)
        {
            joint_setpoint1 = a + b * currT + c * pow(currT, 2) + d * pow(currT, 3);
            speed_target = b + 2 * c * currT + 3 * d * pow(currT, 2);
        }
        else
        {
            sleep_ms(500);
            joint_setpoint1 = 0;
            speed_target = 0;
        }
        printf("%.4f, ", joint_position1);
        printf("%.4f, ", joint_setpoint1);
        printf("%.4f, ", speed_target);
        printf("%.3f \n", v1Filt);
        sleep_ms(30);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
    }
}