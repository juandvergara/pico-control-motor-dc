#include "pico/stdlib.h"
#include <cmath>
#include <stdio.h>
#include <string.h>
#include "dc_motor_v2.h"
#include "encoder.h"
#include "pid_filter.h"

#define SLAVE
#include "pins.h"

#define COMMAND_POS 'p'
#define COMMAND_VEL 'v'
#define READ_ENCODER 'e'
#define COMMAND_POS_VEL 'a'
#define HOME 'h'
#define SET_VEL_MODE 's'

struct Joint
{
    float position = 0;
    float velocity = 0;
    float effort = 0;
    float ref_position = 0;
    float ref_velocity = 0;
};

Joint elbow_joint;
Joint wrist_left_joint;
Joint wrist_right_joint;

Encoder encoder3(M3_ENC_A_PIN, M3_ENC_B_PIN);
Encoder encoder4(M4_ENC_A_PIN, M4_ENC_B_PIN);
Encoder encoder5(M5_ENC_A_PIN, M5_ENC_B_PIN);

DCMotor motor3(M3_ENA_PIN, M3_ENB_PIN);
DCMotor motor4(M4_ENA_PIN, M4_ENB_PIN);
DCMotor motor5(M5_ENA_PIN, M5_ENB_PIN);

float kp1 = 0.2;
float ki1 = 0.00008;
float kd1 = 0.00007;
float kp2 = 0.2;
float ki2 = 0.00008;
float kd2 = 0.00007;

float v1Prev = 0.0;
float v2Prev = 0.0;
float v3Prev = 0.0;

uint32_t sample_time_ms = 10;
float pid_rate;


float result[50];
int user_input;

PID PID_Joint1(&elbow_joint.position, &elbow_joint.velocity, &elbow_joint.effort, &elbow_joint.ref_position, &elbow_joint.ref_velocity,
               kp1, ki1, kd1, sample_time_ms);
PID PID_Joint2(&wrist_left_joint.position, &wrist_left_joint.velocity, &wrist_left_joint.effort, &wrist_left_joint.ref_position, &wrist_left_joint.ref_velocity,
               kp2, ki2, kd2, sample_time_ms);
PID PID_Joint3(&wrist_right_joint.position, &wrist_right_joint.velocity, &wrist_right_joint.effort, &wrist_right_joint.ref_position, &wrist_right_joint.ref_velocity,
               kp2, ki2, kd2, sample_time_ms);

uint32_t millis()
{
    return to_ms_since_boot(get_absolute_time());
}

void set_vel_mode(float mode, bool print_msg)
{
    if (mode == 1.0)
    {
        if (print_msg)
            printf("Velocity control mode on \n");
        PID_Joint1.set_gains(0.0, 0.0, kd1);
        PID_Joint2.set_gains(0.0, 0.0, kd1);
        PID_Joint3.set_gains(0.0, 0.0, kd1);
    }
    else if (mode == 0.0)
    {
        if (print_msg)
            printf("Velocity control mode off \n");
        elbow_joint.ref_position = elbow_joint.position;
        wrist_left_joint.ref_position = wrist_left_joint.position;
        wrist_right_joint.ref_position = wrist_right_joint.position;
        PID_Joint1.set_gains(kp1, ki1, kd1);
        PID_Joint2.set_gains(kp1, ki1, kd1);
        PID_Joint3.set_gains(kp1, ki1, kd1);
    }
    else
    {
        if (print_msg)
            printf("Wrong command! Set 1 to vel mode, 0 to pos mode \n");
    }
}

void print_state_joints() {
    printf("%.4f, ", elbow_joint.position);
    printf("%.4f, ", wrist_left_joint.position - elbow_joint.position);
    printf("%.4f \n", wrist_right_joint.position + elbow_joint.position);
    printf("%.4f, ", elbow_joint.ref_velocity);
    printf("%.4f, ", wrist_left_joint.ref_velocity - elbow_joint.ref_velocity);
    printf("%.4f \n", wrist_right_joint.ref_velocity + elbow_joint.ref_velocity);
    // printf("Entradas recibidas");
    // printf("Effort: J1: %.3f, J2: %.3f, J3: %.3f \n", elbow_joint.effort, wrist_left_joint.effort, wrist_right_joint.effort);
    // printf("Position: J1: %.3f, J2: %.3f \n", elbow_joint.position, wrist_left_joint.position);
    // printf("Ef1: %.4f, \n", elbow_joint.effort);
}

void home()
{
    gpio_init(M3_HOME_SW);
    gpio_pull_up(M3_HOME_SW);
    gpio_init(M4_HOME_SW);
    gpio_pull_up(M4_HOME_SW);
    gpio_init(M5_HOME_SW);
    gpio_pull_up(M5_HOME_SW);
    sleep_ms(100);

    bool elbow_home = false;
    bool wrist_pitch = false;
    bool wrist_yaw = false;

    encoder3.encoder_pos = 0;
    encoder4.encoder_pos = 0;
    encoder5.encoder_pos = 0;

    while (!elbow_home)
    {
        if (!gpio_get(M3_HOME_SW))
        {
            printf("Fixing elbow home \n");
            motor3.write(0.4);
            sleep_ms(1000);
        }
        else
        {
            while (true)
            {
                motor3.write(-0.4);
                if (!gpio_get(M3_HOME_SW))
                {
                    motor3.write(0.0);
                    encoder3.encoder_pos = 0;
                    elbow_joint.position = 0;
                    elbow_joint.velocity = 0;
                    printf("Home elbow success! \n");
                    break;
                }
                printf("Homing elbow... \n");
            }
            elbow_home = true;
        }
    }

    while (!wrist_yaw)
    {
        if (!gpio_get(M5_HOME_SW))
        {
            printf("Fixing wrist yaw home, \n");
            motor4.write(0.4);
            motor5.write(-0.4);
            sleep_ms(1000);
        }
        else
        {
            while (true)
            {
                motor5.write(0.4);
                motor4.write(-0.4);
                if (!gpio_get(M5_HOME_SW))
                {
                    printf("Homing wrist yaw..., \n");
                    motor5.write(0.0);
                    motor4.write(0.0);
                    encoder4.encoder_pos = 0;
                    encoder5.encoder_pos = 0;
                    wrist_right_joint.position = 0;
                    wrist_right_joint.velocity = 0;
                    wrist_left_joint.position = 0;
                    wrist_left_joint.velocity = 0;
                    printf("Home wrist yaw success! , \n");
                    break;
                }
            }
            wrist_yaw = true;
        }
    }

    while (!wrist_pitch)
    {
        if (!gpio_get(M4_HOME_SW))
        {
            printf("Fixing wrist pitch home, \n");
            motor4.write(-0.4);
            motor5.write(-0.4);
            sleep_ms(1000);
        }
        else
        {
            while (true)
            {
                motor4.write(0.4);
                motor5.write(0.4);
                if (!gpio_get(M4_HOME_SW))
                {
                    motor4.write(0.0);
                    motor5.write(0.0);
                    encoder4.encoder_pos = -1820;
                    encoder5.encoder_pos = -2200;
                    wrist_left_joint.position = 0;
                    wrist_left_joint.velocity = 0;
                    wrist_right_joint.position = 0;
                    wrist_right_joint.velocity = 0;
                    wrist_left_joint.ref_position = 0;
                    wrist_right_joint.ref_position = 0;
                    printf("Home wrist pitch success! , \n");
                    sleep_ms(100);
                    break;
                }
                printf("Homing wrist pitch..., \n");
            }
            wrist_pitch = true;
        }
    }
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

        for (int i = 0; i < 2; i++)
        {
            result[i + 1] = strtof(previous + 1, &current);
            previous = current;
            printf(", %.3f", result[i + 1]);
        }
        printf("\n");
        elbow_joint.ref_position = result[0];
        wrist_left_joint.ref_position = result[1] + elbow_joint.ref_position;
        wrist_right_joint.ref_position = -result[2] - elbow_joint.ref_position;
        break;

    case (COMMAND_VEL):
        printf("Set velocity goal was call\n");
        token = strtok(NULL, " ");

        result[0] = strtof(token, &previous);
        printf("%.1f", result[0]);

        for (int i = 0; i < size_buffer - 1; i++)
        {
            result[i + 1] = strtof(previous + 1, &current);
            previous = current;
            printf(", %.1f", result[i + 1]);
        }
        printf("\n");
        elbow_joint.ref_velocity = result[0];
        wrist_left_joint.ref_velocity = result[1] + elbow_joint.ref_velocity;
        wrist_right_joint.ref_velocity = -result[2] - elbow_joint.ref_velocity;
        break;
    case (READ_ENCODER):

        printf("Encoder callback \n");
        print_state_joints();
        break;
    case (HOME):

        printf("Homing... \n");
        home();
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
    motor3.write(0.0);
    motor4.write(0.0);
    motor5.write(0.0);
    PID_Joint1.set_output_limits(-1.0f, 1.0f);
    PID_Joint2.set_output_limits(-1.0f, 1.0f);
    PID_Joint3.set_output_limits(-1.0f, 1.0f);
    elbow_joint.ref_position = 0;
    wrist_left_joint.ref_position = 0;
    wrist_right_joint.ref_position = 0;
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

    float position_elbow = float(joint1_ticks) * 360.0f / (80.0f * 127.7f * 4.0f);
    float position_wrist_left = float(joint2_ticks) * 360.0f * 23.0f / (80.0f * 65.5f * 32.0f);
    float position_wrist_right = float(joint3_ticks) * 360.0f * 23.0f / (80.0f * 65.5f * 32.0f);

    float velocity_elbow = (position_elbow - elbow_joint.position) / pid_rate;
    float velocity_wrist_left = (position_wrist_left - wrist_left_joint.position) / pid_rate;
    float velocity_wrist_right = (position_wrist_right - wrist_right_joint.position) / pid_rate;

    elbow_joint.position = position_elbow;
    wrist_left_joint.position = position_wrist_left;
    wrist_right_joint.position = position_wrist_right;

    elbow_joint.velocity = 0.854 * elbow_joint.velocity + 0.0728 * velocity_elbow + 0.0728 * v1Prev;
    wrist_left_joint.velocity = 0.854 * wrist_left_joint.velocity + 0.0728 * velocity_wrist_left + 0.0728 * v2Prev;
    wrist_right_joint.velocity = 0.854 * wrist_right_joint.velocity + 0.0728 * velocity_wrist_right + 0.0728 * v3Prev;

    v1Prev = velocity_elbow;
    v2Prev = velocity_wrist_left;
    v3Prev = velocity_wrist_right;

    PID_Joint1.compute();
    PID_Joint2.compute();
    PID_Joint3.compute();

    motor1_vel = elbow_joint.effort;
    motor2_vel = wrist_left_joint.effort;
    motor3_vel = wrist_right_joint.effort;

    motor3.write(-motor1_vel);
    motor4.write(motor2_vel);
    motor5.write(-motor3_vel);
}

bool controller_timer_callback(repeating_timer_t *rt)
{
    updatePid(int32_t(encoder3.encoder_pos), int32_t(encoder4.encoder_pos), int32_t(encoder5.encoder_pos));
    return true;
}

void encoders_callback(uint gpio, uint32_t events)
{
    encoder3.readPosition();
    encoder4.readPosition();
    encoder5.readPosition();
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

    while ( (current_time - initial_time) < t_final)
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

    gpio_set_irq_enabled_with_callback(M3_ENC_A_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoders_callback);
    gpio_set_irq_enabled_with_callback(M4_ENC_A_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoders_callback);
    gpio_set_irq_enabled_with_callback(M5_ENC_A_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoders_callback);

    // home();

    repeating_timer_t timer;
    if (!add_repeating_timer_ms(-sample_time_ms, controller_timer_callback, NULL, &timer))
    {
        printf("Failure by not set timer!! \n");
    }

    sleep_ms(1000);

    path_generator(90, 30, &elbow_joint);
    path_generator(0, 30, &elbow_joint);

    while (true)
    {
        user_input = getchar_timeout_us(0); // Esperar la entrada del usuario
        process_user_input(user_input);
    }
}