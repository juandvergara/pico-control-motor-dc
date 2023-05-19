#include "pico/stdlib.h"
#include <cmath>
#include <stdio.h>
#include <string.h>
#include "dc_motor_v2.h"
#include "encoder.h"
#include "pid_v3.h"

#define MASTER
#include "pins.h"

#define COMMAND_POS 'p'
#define BASE_RELATION 0.007047768206734534f  

#define ENCODER_A_MOTOR M1_ENC_A_PIN
#define ENCODER_B_MOTOR M1_ENC_B_PIN
#define MOTOR_ENA M1_ENA_PIN
#define MOTOR_ENB M1_ENB_PIN

struct Joint
{
    float position = 0;
    float velocity = 0;
    float effort = 0;
    float ref_position = 0;
    float ref_velocity = 0;
};

Joint probe_joint;

Encoder encoder_joint(ENCODER_A_MOTOR, ENCODER_B_MOTOR);

DCMotor motor_joint(MOTOR_ENA, MOTOR_ENB);

float kp = 0.510;
float ki = 1.015;
float kd = 0.016;

float k_h = 0.01;
float k_gamma = 0.8;

float v1Prev = 0.0;

uint32_t sample_time_ms = 5;
float pid_rate;

float result[50];
int user_input;

PID_V3 PID_Joint(&probe_joint.position, &probe_joint.velocity, &probe_joint.effort, &probe_joint.ref_position, &probe_joint.ref_velocity,
               kp, ki, kd, sample_time_ms, k_h, k_gamma);


uint32_t millis()
{
    return to_ms_since_boot(get_absolute_time());
}

void print_state_joints()
{
    // printf("Entradas recibidas");
    // printf("Effort: J1: %.3f, J2: %.3f, J3: %.3f \n", probe_joint.effort, wrist_left_joint.effort, wrist_right_joint.effort);
    // printf("Position: J1: %.3f, J2: %.3f \n", probe_joint.position, wrist_left_joint.position);
    // printf("Ef1: %.4f, \n", probe_joint.effort);
}


void command_callback(char *buffer, int size_buffer)
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
        printf("%.3f", result[0]);

        printf("\n");
        probe_joint.ref_position = result[0];
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
            int size = ((sizeof(in_buffer) / sizeof(in_buffer[0])) - 2) / 8;
            command_callback(in_buffer, size);
            break;
        }
        input_std = getchar_timeout_us(0);
    }
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
}

void initRobot()
{
    motor_joint.write(0.0);
    PID_Joint.set_output_limits(-1.0f, 1.0f);
    probe_joint.ref_position = 0;
    pid_rate = float(sample_time_ms) / 1000.0f;
}

void updatePid(int32_t joint_encoder_ticks)
{
    int32_t joint_ticks = joint_encoder_ticks;

    float motor_vel = 0;

    float position_joint = float(joint_ticks) * 360.0f / (80.0f * 127.7f * 4.0f);

    float velocity_joint = (position_joint - probe_joint.position) / pid_rate;

    probe_joint.position = position_joint;

    probe_joint.velocity = 0.854 * probe_joint.velocity + 0.0728 * velocity_joint + 0.0728 * v1Prev;

    v1Prev = velocity_joint;

    PID_Joint.compute();

    motor_vel = probe_joint.effort;

    motor_joint.write(motor_vel);
}

bool controller_timer_callback(repeating_timer_t *rt)
{
    updatePid(int32_t(encoder_joint.encoder_pos));
    return true;
}

void encoders_callback(uint gpio, uint32_t events)
{
    encoder_joint.readPosition();
}

void path_generator(float target, float deg_s, Joint *joint)
{
    float initial_time = millis() / 1e3;

    float theta0 = (*joint).position;
    float dot_theta0 = 0;
    float thetaf = target;
    float dot_thetaf = 0;
    float t_final = abs(target - theta0) / deg_s;

    float a = theta0;
    float b = dot_theta0;
    float c = 3.0 * (thetaf - theta0) / pow(t_final, 2) - 2 * dot_theta0 / pow(t_final, 2) - dot_thetaf / pow(t_final, 2);
    float d = -2.0 * (thetaf - theta0) / pow(t_final, 3) + (dot_thetaf - dot_theta0) / pow(t_final, 2);

    float current_time = millis() / 1e3;

    while ((current_time - initial_time) < t_final)
    {
        float time_process = current_time - initial_time;
        (*joint).ref_position = a + b * time_process + c * pow(time_process, 2) + d * pow(time_process, 3);
        (*joint).ref_velocity = b + 2 * c * time_process + 3 * d * pow(time_process, 2);
        current_time = millis() / 1e3;
    }

    printf("End path SUCCESS \n");
}
int main()
{
    stdio_init_all();
    printf("PID Motor test");
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    initRobot();

    gpio_set_irq_enabled_with_callback(ENCODER_A_MOTOR, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoders_callback);

    repeating_timer_t timer;
    if (!add_repeating_timer_ms(-sample_time_ms, controller_timer_callback, NULL, &timer))
    {
        printf("Failure by not set timer!! \n");
    }

    float theta0 = 0;
    float dot_theta0 = 0;
    float thetaf = round(50 / BASE_RELATION) * BASE_RELATION;
    float dot_thetaf = 0;
    float t_final = 5.0;

    float a = theta0;
    float b = dot_theta0;
    float c = 3.0 * (thetaf - theta0) / pow(t_final, 2) - 2 * dot_theta0 / pow(t_final, 2) - dot_thetaf / pow(t_final, 2);
    float d = -2.0 * (thetaf - theta0) / pow(t_final, 3) + (dot_thetaf - dot_theta0) / pow(t_final, 2);

    int input_char;
    int input_char_index;
    char *char_pt1;
    int i = 0;
    float res = 0;
    float MSE = 0;
    float joint1_sp = 0.0;

    sleep_ms(3000);

    while (true)
    {
        user_input = getchar_timeout_us(0); // Esperar la entrada del usuario
        process_user_input(user_input);  

        sleep_ms(20);
        float currT = millis() / 1e3 - 3;
        if (currT < t_final)
        {
            probe_joint.ref_position = a + b * currT + c * pow(currT, 2) + d * pow(currT, 3);
            probe_joint.ref_velocity = b + 2 * c * currT + 3 * d * pow(currT, 2);
            MSE += pow((probe_joint.velocity - probe_joint.ref_velocity), 2);
            i++;
        }
        else
        {
            res = MSE / ((float)i);
            // probe_joint.ref_position = 0;
            // probe_joint.ref_velocity = 0;
            sleep_ms(10);
        }
        printf("%.4f, ", res);
        printf("%.4f, ", probe_joint.position);
        printf("%.4f, ", probe_joint.ref_position);
        printf("%.4f, ", probe_joint.ref_velocity);
        printf("%.3f \n", probe_joint.velocity);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
    }
}