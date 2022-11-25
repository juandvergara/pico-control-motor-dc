#include "pico/stdlib.h"
#include <cmath>
#include <stdio.h>
#include "dc_motor_v2.h"
#include "encoder.h"
#include "pid_filter.h"

#define elbow_sw 7

Encoder encoder3(M3_ENC_A_PIN, M3_ENC_B_PIN);

DCMotor motor3(M3_ENA_PIN, M3_ENB_PIN);

bool homing_success = false;

struct Joint
{
    float position = 0;
    float velocity = 0;
    float effort = 0;
    float ref_position = 0;
    float ref_velocity = 0;
};

Joint elbow_joint;

float kp1 = 0.2;
float ki1 = 0.00008;
float kd1 = 0.00007;

float v1Prev = 0;

uint32_t sample_time_ms = 10;
float pid_rate;

char in_buffer[500];
uint16_t char_idx = 0;

PID PID_Joint1(&elbow_joint.position, &elbow_joint.velocity, &elbow_joint.effort, &elbow_joint.ref_position, &elbow_joint.ref_velocity,
               kp1, ki1, kd1, sample_time_ms);

uint32_t millis()
{
    return to_ms_since_boot(get_absolute_time());
}

void home()
{
    gpio_init(elbow_sw);
    gpio_pull_up(elbow_sw);
    sleep_ms(100);

    while (true)
    {
        if (!gpio_get(elbow_sw))
        {
            printf("fixing home, \n");
            motor3.write(0.4);
            sleep_ms(1000);
        }
        else
        {
            motor3.write(-0.4);
            if (!gpio_get(elbow_sw))
            {
                printf("Homing elbow..., \n");
                motor3.write(0.0);
                elbow_joint.position = 0;
                elbow_joint.velocity = 0;
                printf("Home elbow success! , \n");
                break;
            }
        }
    }
}

void initRobot()
{
    home();
    motor3.write(0.0);
    PID_Joint1.set_output_limits(-1.0f, 1.0f);
    elbow_joint.ref_position = 0;
    elbow_joint.ref_velocity = 0;
    pid_rate = float(sample_time_ms) / 1000.0f;
}

void updatePid(int32_t joint1_encoder_ticks)
{
    int32_t joint1_ticks = joint1_encoder_ticks;

    float motor1_vel = 0;
    float position = float(joint1_ticks) * 360.0f / (80.0f * 127.7f * 4.0f);
    float velocity = (position - elbow_joint.position) / pid_rate;

    elbow_joint.position = position;
    elbow_joint.velocity = 0.854 * elbow_joint.velocity + 0.0728 * velocity + 0.0728 * v1Prev;
    v1Prev = velocity;

    PID_Joint1.compute();

    motor1_vel = elbow_joint.effort;

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

float path_generator(float target, float time, float currT, bool pos)
{
    float result;

    float theta0 = 0;
    float dot_theta0 = 0;
    float thetaf = target;
    float dot_thetaf = 0;
    float t_final = time;

    float a = theta0;
    float b = dot_theta0;
    float c = 3.0 * (thetaf - theta0) / pow(t_final, 2) - 2 * dot_theta0 / pow(t_final, 2) - dot_thetaf / pow(t_final, 2);
    float d = -2.0 * (thetaf - theta0) / pow(t_final, 3) + (dot_thetaf - dot_theta0) / pow(t_final, 2);

    if (pos)
    {
        result = a + b * currT + c * pow(currT, 2) + d * pow(currT, 3);
    }
    else
    {
        result = b + 2 * c * currT + 3 * d * pow(currT, 2);
    }
    return result;
}

int main()
{
    motor3.write(0.0);
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

    int input_char;
    int input_char_index;
    char *char_pt1;
    int i = 0;
    float res = 0;
    float MSE = 0;
    float joint1_sp = 0.0;

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
            elbow_joint.ref_position = joint1_sp;
        }
        sleep_ms(20);
        float currT = millis() / 1e3;
        printf("%.4f, ", elbow_joint.position);
        printf("%.4f, ", elbow_joint.ref_position);
        printf("%.4f, ", elbow_joint.ref_velocity);
        printf("%.3f \n", elbow_joint.velocity);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
    }
}