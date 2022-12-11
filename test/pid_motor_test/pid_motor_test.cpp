#include "pico/stdlib.h"
#include <cmath>
#include <stdio.h>
#include "dc_motor_v2.h"
#include "encoder.h"
#include "pid_filter.h"

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

char in_buffer[500] = {};
uint16_t char_idx = 0;

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

bool timerCallback(repeating_timer_t *rt)
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

    home();

    repeating_timer_t timer;
    if (!add_repeating_timer_ms(-sample_time_ms, timerCallback, NULL, &timer))
    {
        printf("Failure by not set timer!! \n");
    }

    int input_char;
    int input_char_index;
    char *char_pt1;
    char *char_pt2;
    char *char_pt3;

    float joint1_sp = 0.0;
    float joint2_sp = 0.0;
    float joint3_sp = 0.0;

    while (true)
    {
        input_char = getchar_timeout_us(0); // Esperar la entrada del usuario
        while (input_char != PICO_ERROR_TIMEOUT)
        {
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            putchar(input_char);                        // Print user input in console
            in_buffer[input_char_index++] = input_char; // Index user input to buffer array
            if (input_char == '/')
            {
                in_buffer[input_char_index] = 0; // end of string
                input_char_index = 0;
                joint1_sp = strtof(in_buffer, &char_pt1);    // Conversion string (char) to float
                joint2_sp = strtof(char_pt1 + 1, &char_pt2); // Conversion string (char) to float
                joint3_sp = strtof(char_pt2 + 1, &char_pt3); // Add 1 to bring up the comma
                break;
            }
            elbow_joint.ref_position = joint1_sp;
            wrist_left_joint.ref_position = joint2_sp + elbow_joint.ref_position;
            wrist_right_joint.ref_position = -joint3_sp - elbow_joint.ref_position;
            input_char = getchar_timeout_us(0);
        }
        // gpio_put(LED_PIN, false);
        // printf("Entradas recibidas");
        // printf("Effort: J1: %.3f, J2: %.3f, J3: %.3f \n", elbow_joint.effort, wrist_left_joint.effort, wrist_right_joint.effort);
        // printf("Position: J1: %.3f, J2: %.3f \n", elbow_joint.position, wrist_left_joint.position);
        // printf("Ef1: %.4f, \n", elbow_joint.effort);
        printf("%.4f, ", elbow_joint.position);
        printf("%.4f, ", wrist_left_joint.position - elbow_joint.position);
        printf("%.4f \n", wrist_right_joint.position + elbow_joint.position); /*
         printf("Pos2: %.4f\n \n", wrist_left_joint.position);*/
        sleep_ms(10);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
    }
}