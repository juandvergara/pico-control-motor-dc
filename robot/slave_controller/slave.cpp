#include "pico/stdlib.h"
#include <cmath>
#include <stdio.h>
#include "dc_motor_v2.h"
#include "encoder.h"
#include "pid_controller.h"

#define ELBOW_RELATION 0.008809710258418167 // 360.0f / (80.0f * 127.7f * 4.0f)
#define WRIST_RELATION 0.049379770992366415 // 360.0f * 23.0f / (80.0f * 65.5f * 32.0f)

DCMotor elbow_motor(M0_ENA_PIN, M0_ENB_PIN);
DCMotor wrist_left_motor(M1_ENA_PIN, M1_ENB_PIN);
DCMotor wrist_right_motor(M2_ENA_PIN, M2_ENB_PIN);

Encoder elbow_encoder(M0_ENC_A_PIN, M0_ENC_B_PIN);
Encoder wrist_left_encoder(M1_ENC_A_PIN, M1_ENC_B_PIN);
Encoder wrist_right_encoder(M2_ENC_A_PIN, M2_ENC_B_PIN);

float kp1 = 0.5;
float kd1 = 0.0;
float ki1 = 0.2;
float kp2 = 1.0;
float kd2 = 0.0;
float ki2 = 0.0;

float elbow_input, elbow_effort, elbow_setpoint = 0.0;
float wrist_left_input, wrist_left_effort, wrist_left_setpoint = 0.0;
float wrist_right_input, wrist_right_effort, wrist_right_setpoint = 0.0;

float elbow_position, wrist_left_position, wrist_right_position;

uint32_t sample_time_ms = 20;
float pid_rate;

PID PID_elbow(&elbow_input, &elbow_effort, &elbow_setpoint, kp1, ki1, kd1, sample_time_ms);
PID PID_wrist_left(&wrist_left_input, &wrist_left_effort, &wrist_left_setpoint, kp2, ki2, kd2, sample_time_ms);
PID PID_wrist_right(&wrist_right_input, &wrist_right_effort, &wrist_right_setpoint, kp2, ki2, kd2, sample_time_ms);

uint32_t millis()
{
    return to_ms_since_boot(get_absolute_time());
}

void initRobot()
{
    elbow_motor.write(0.0);
    wrist_left_motor.write(0.0);
    wrist_right_motor.write(0.0);

    PID_elbow.set_output_limits(-1.0f, 1.0f);
    PID_wrist_left.set_output_limits(-1.0f, 1.0f);
    PID_wrist_right.set_output_limits(-1.0f, 1.0f);

    elbow_setpoint = 0;
    wrist_left_setpoint = 0;
    wrist_right_setpoint = 0;

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

    elbow_position = float(joint1_ticks) * ELBOW_RELATION;
    wrist_left_position = float(joint2_ticks) * WRIST_RELATION;
    wrist_right_position = float(joint3_ticks) * WRIST_RELATION;

    elbow_input = elbow_position;
    wrist_left_input = wrist_left_position;
    wrist_right_input = wrist_right_position;

    PID_elbow.compute();
    PID_wrist_left.compute();
    PID_wrist_right.compute();

    motor1_vel = elbow_effort;
    motor2_vel = wrist_left_effort;
    motor3_vel = wrist_right_effort;

    elbow_motor.write(motor1_vel);
    wrist_left_motor.write(motor2_vel);
    wrist_right_motor.write(motor3_vel);
}

bool timerCallback(repeating_timer_t *rt)
{
    updatePid(int32_t(elbow_encoder.encoder_pos), int32_t(wrist_left_encoder.encoder_pos), int32_t(wrist_right_encoder.encoder_pos));
    return true;
}

void encoders_callback(uint gpio, uint32_t events)
{
    elbow_encoder.readPosition();
    wrist_left_encoder.readPosition();
    wrist_right_encoder.readPosition();
}

int main()
{
    stdio_init_all();
    printf("PID Motor test");
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    initRobot();

    gpio_set_irq_enabled_with_callback(M1_ENC_A_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoders_callback);
    gpio_set_irq_enabled_with_callback(M2_ENC_A_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoders_callback);
    gpio_set_irq_enabled_with_callback(M3_ENC_A_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoders_callback);

    repeating_timer_t timer;
    if (!add_repeating_timer_ms(-sample_time_ms, timerCallback, NULL, &timer))
    {
        printf("Failure by not set timer!! \n");
    }

    char in_buffer[100];
    int input_char;
    int input_char_index;
    char *char_pt1;
    char *char_pt2;
    char *char_pt3;

    float elbow_sp = 0.0;
    float wrist_left_sp = 0.0;
    float wrist_right_sp = 0.0;

    while (true)
    {

        input_char = getchar_timeout_us(0);
        while (input_char != PICO_ERROR_TIMEOUT)
        {
            gpio_put(PICO_DEFAULT_LED_PIN, 1);          // Print user input in console
            in_buffer[input_char_index++] = input_char; // Index user input to buffer array
            if (input_char == '/')
            {
                in_buffer[input_char_index] = 0;
                input_char_index = 0;
                elbow_sp = strtof(in_buffer, &char_pt1);   // Conversion string (char) to float
                wrist_left_sp = strtof(char_pt1 + 1, &char_pt2);     // Conversion string (char) to float
                wrist_right_sp = strtof(char_pt2 + 1, &char_pt3); // Add 1 to bring up the comma
                break;
            }
            elbow_setpoint = elbow_sp;
            wrist_left_setpoint = wrist_left_sp;
            wrist_right_setpoint = wrist_right_sp;
            input_char = getchar_timeout_us(0);
            printf("\n Caracter recibido \n");
        }
        printf("Slide base: sp %.3f, pos: %.3f, \n", elbow_setpoint, elbow_position);
        printf("Base: sp %.3f, pos: %.3f, \n", wrist_left_setpoint, wrist_left_position);
        printf("Shoulder: sp %.3f, pos: %.3f\n \n", wrist_right_setpoint, wrist_right_position);
        sleep_ms(500);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
    }
}