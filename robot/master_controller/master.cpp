#include <cmath>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "dc_motor_v2.h"
#include "encoder.h"
#include "pid_filter.h"
#include "command.h"

#define MASTER
#include "pins.h"

#define SLIDEBASE_RELATION 0.008809710258418167f // 360.0f / (80.0f * 127.7f * 4.0f)
#define BASE_RELATION 0.007047768206734534f      // 360.0f / (80.0f * 127.7f * 5.0f)
#define SHOULDER_RELATION 0.008809710258418167f  // 360.0f / (80.0f * 127.7f * 4.0f)

static int SLAVE_ADDR = 0x15;

struct Joint
{
    float position = 0;
    float velocity = 0;
    float effort = 0;
    float ref_position = 0;
    float ref_velocity = 0;
};

Joint slidebase_joint;
Joint base_joint;
Joint shoulder_joint;

DCMotor slidebase_motor(M0_ENA_PIN, M0_ENB_PIN);
DCMotor base_motor(M1_ENA_PIN, M1_ENB_PIN);
DCMotor shoulder_motor(M2_ENA_PIN, M2_ENB_PIN);

Encoder slidebase_encoder(M0_ENC_A_PIN, M0_ENC_B_PIN);
Encoder base_encoder(M1_ENC_A_PIN, M1_ENC_B_PIN);
Encoder shoulder_encoder(M2_ENC_A_PIN, M2_ENC_B_PIN);

/*float PID_param1[3] = {2.0, 0.5, 0.5};
float PID_param2[3] = {2.0, 0.5, 0.5};*/

float kp1 = 0.2;
float ki1 = 0.00008;
float kd1 = 0.00007;
float kp2 = 0.2;
float ki2 = 0.00008;
float kd2 = 0.00007;

float v1Prev = 0.0;
float v2Prev = 0.0;
float v3Prev = 0.0;

float result[50];
uint8_t command_slave;

uint8_t *target_slave1, *target_slave2, *target_slave3;
uint8_t *status_slidebase, *status_base, *status_shoulder;

uint32_t sample_time_ms = 10;
float pid_rate;

PID PID_slidebase(&slidebase_joint.position, &slidebase_joint.velocity, &slidebase_joint.effort, &slidebase_joint.ref_position, &slidebase_joint.ref_velocity,
                  kp1, ki1, kd1, sample_time_ms);
PID PID_base(&base_joint.position, &base_joint.velocity, &base_joint.effort, &base_joint.ref_position, &base_joint.ref_velocity,
             kp2, ki2, kd2, sample_time_ms);
PID PID_shoulder(&shoulder_joint.position, &shoulder_joint.velocity, &shoulder_joint.effort, &shoulder_joint.ref_position, &shoulder_joint.ref_velocity,
                 kp2, ki2, kd2, sample_time_ms);

uint32_t millis()
{
    return to_ms_since_boot(get_absolute_time());
}

void set_vel_mode(float mode, bool print_msg)
{
    if (mode == 1.0)
    {
        command_slave = SET_VEL_MODE;
        if (print_msg)
            printf("Velocity control mode on \n");
        PID_slidebase.set_gains(0.0, 0.0, kd1);
        PID_base.set_gains(0.0, 0.0, kd1);
        PID_shoulder.set_gains(0.0, 0.0, kd1);
    }
    else if (mode == 0.0)
    {
        command_slave = UNSET_VEL_MODE;
        if (print_msg)
            printf("Velocity control mode off \n");
        slidebase_joint.ref_position = slidebase_joint.position;
        base_joint.ref_position = base_joint.position;
        shoulder_joint.ref_position = shoulder_joint.position;
        PID_slidebase.set_gains(kp1, ki1, kd1);
        PID_base.set_gains(kp1, ki1, kd1);
        PID_shoulder.set_gains(kp1, ki1, kd1);
    }
    else
    {
        if (print_msg)
            printf("Wrong command! Set 1 to vel mode, 0 to pos mode \n");
    }
}

void print_state_joints()
{
    printf("%.3f,%.3f,%.3f \n",
           slidebase_joint.position, base_joint.position, shoulder_joint.position);
    /*printf("Slide base: sp %.3f, pos: %.3f, \n", slidebase_joint.ref_position, slidebase_joint.position);
    printf("Base: sp %.3f, pos: %.3f, \n", base_joint.ref_position, base_joint.position);
    printf("Shoulder: sp %.3f, pos: %.3f\n \n", shoulder_joint.ref_position, shoulder_joint.position);*/
}

void send_info_slave(float *result, bool pos_mode)
{
    // printf("Sending to slave %.5f, %.5f, %.5f \n", result[3], result[4], result[5]);
    target_slave1 = (uint8_t *)(&result[3]);
    target_slave2 = (uint8_t *)(&result[4]);
    target_slave3 = (uint8_t *)(&result[5]);

    if (pos_mode)
    {
        status_slidebase = (uint8_t *)(&slidebase_joint.position);
        status_base = (uint8_t *)(&base_joint.position);
        status_shoulder = (uint8_t *)(&shoulder_joint.position);
    }
    else
    {
        status_slidebase = (uint8_t *)(&slidebase_joint.velocity);
        status_base = (uint8_t *)(&base_joint.velocity);
        status_shoulder = (uint8_t *)(&shoulder_joint.velocity);
    }
}

void command_callback(char *buffer)
{
    char *current;
    char *previous;
    char *token = strtok(buffer, " ");
    char command = *token;

    switch (command)
    {
    case (COMMAND_POS):
        printf("Set position goal was call\n");
        token = strtok(NULL, " ");

        result[0] = strtof(token, &previous);

        for (int i = 0; i < 2; i++)
        {
            result[i + 1] = strtof(previous + 1, &current);
            previous = current;
        }

        slidebase_joint.ref_position = result[0];
        base_joint.ref_position = -round(result[1] / BASE_RELATION) * BASE_RELATION;
        shoulder_joint.ref_position = round(result[2] / SHOULDER_RELATION) * SHOULDER_RELATION;
        break;

    case (COMMAND_VEL):
        printf("Set velocity goal was call\n");
        token = strtok(NULL, " ");

        result[0] = strtof(token, &previous);
        printf("%.1f", result[0]);

        for (int i = 0; i < 2 - 1; i++)
        {
            result[i + 1] = strtof(previous + 1, &current);
            previous = current;
            printf(", %.1f", result[i + 1]);
        }
        printf("\n");
        slidebase_joint.ref_velocity = result[0];
        base_joint.ref_velocity = result[1];
        shoulder_joint.ref_velocity = result[2];
        break;
    case (READ_ENCODER):

        printf("Encoder callback \n");
        print_state_joints();
        break;
    case (SET_VEL_MODE):
        token = strtok(NULL, " ");
        float mode;
        mode = strtof(token, &previous);
        set_vel_mode(mode, true);
        break;
    default:
        printf("Invalid command \n");
        break;
    }
}

void process_user_input(int input_std)
{
    int input_char_index;
    char in_buffer[50];
    while (input_std != PICO_ERROR_TIMEOUT)
    {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        in_buffer[input_char_index++] = input_std;
        if (input_std == '\n')
        {
            in_buffer[input_char_index] = 0;
            input_char_index = 0;
            command_callback(in_buffer);
            break;
        }
        input_std = getchar_timeout_us(0);
    }
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
}

void initRobot()
{
    slidebase_motor.write(0.0);
    base_motor.write(0.0);
    shoulder_motor.write(0.0);

    PID_slidebase.set_output_limits(-1.0f, 1.0f);
    PID_base.set_output_limits(-1.0f, 1.0f);
    PID_shoulder.set_output_limits(-1.0f, 1.0f);

    slidebase_joint.ref_position = 0;
    base_joint.ref_position = 0;
    shoulder_joint.ref_position = 0;

    pid_rate = float(sample_time_ms) / 1000.0f;
}

void updatePid(int32_t joint1_encoder_ticks, int32_t joint2_encoder_ticks, int32_t joint3_encoder_ticks)
{
    int32_t joint1_ticks = joint1_encoder_ticks;
    int32_t joint2_ticks = joint2_encoder_ticks;
    int32_t joint3_ticks = joint3_encoder_ticks;

    float motor1_vel = 0;
    float motor2_vel = 0;
    float motor3_vel = 0;

    float position_slidebase = float(joint1_ticks) * SLIDEBASE_RELATION;
    float position_base = float(joint2_ticks) * BASE_RELATION;
    float position_shoulder = float(joint3_ticks) * SHOULDER_RELATION;

    float velocity_slidebase = (position_slidebase - slidebase_joint.position) / pid_rate;
    float velocity_base = (position_base - base_joint.position) / pid_rate;
    float velocity_shoulder = (position_shoulder - shoulder_joint.position) / pid_rate;

    slidebase_joint.position = position_slidebase;
    base_joint.position = position_base;
    shoulder_joint.position = position_shoulder;

    slidebase_joint.velocity = 0.854 * slidebase_joint.velocity + 0.0728 * velocity_slidebase + 0.0728 * v1Prev;
    base_joint.velocity = 0.854 * base_joint.velocity + 0.0728 * velocity_base + 0.0728 * v2Prev;
    shoulder_joint.velocity = 0.854 * shoulder_joint.velocity + 0.0728 * velocity_shoulder + 0.0728 * v3Prev;

    v1Prev = velocity_slidebase;
    v2Prev = velocity_base;
    v3Prev = velocity_shoulder;

    PID_slidebase.compute();
    PID_base.compute();
    PID_shoulder.compute();

    M0_ENC_INVERTED ? motor1_vel = -slidebase_joint.effort : motor3_vel = slidebase_joint.effort;
    M1_ENC_INVERTED ? motor2_vel = -base_joint.effort : motor3_vel = base_joint.effort;
    M2_ENC_INVERTED ? motor3_vel = -shoulder_joint.effort : motor3_vel = shoulder_joint.effort;

    slidebase_motor.write(motor1_vel);
    base_motor.write(motor2_vel);
    shoulder_motor.write(motor3_vel);
}

bool timerCallback(repeating_timer_t *rt)
{
    updatePid(int32_t(slidebase_encoder.encoder_pos), int32_t(base_encoder.encoder_pos), int32_t(shoulder_encoder.encoder_pos));
    return true;
}

void encoders_callback(uint gpio, uint32_t events)
{
    slidebase_encoder.readPosition();
    base_encoder.readPosition();
    shoulder_encoder.readPosition();
}

int main()
{
    stdio_init_all();
    printf("Master control");

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    initRobot();

    gpio_set_irq_enabled_with_callback(M0_ENC_A_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoders_callback);
    gpio_set_irq_enabled_with_callback(M1_ENC_A_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoders_callback);
    gpio_set_irq_enabled_with_callback(M2_ENC_A_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoders_callback);

    repeating_timer_t timer;
    if (!add_repeating_timer_ms(-sample_time_ms, timerCallback, NULL, &timer))
    {
        printf("Failure by not set timer!! \n");
    }

    int input_char;

    while (true)
    {
        input_char = getchar_timeout_us(0);
        process_user_input(input_char);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(10);
    }
}