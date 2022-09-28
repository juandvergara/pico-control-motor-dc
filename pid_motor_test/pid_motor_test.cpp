#include "pico/stdlib.h"
#include <cmath>
#include <stdio.h>
#include "dc_motor.h"
#include "encoder.h"
#include "pid_controller.h"

Encoder encoder1(M1_ENC_A_PIN, M1_ENC_B_PIN);
Encoder encoder2(M2_ENC_A_PIN, M2_ENC_B_PIN);
Encoder encoder3(M3_ENC_A_PIN, M3_ENC_B_PIN);

DCMotor motor1(M1_ENA_PIN, M1_ENB_PIN, M1_PWM_PIN);
DCMotor motor2(M2_ENA_PIN, M2_ENB_PIN, M2_PWM_PIN);
DCMotor motor3(M3_ENA_PIN, M3_ENB_PIN, M3_PWM_PIN);

float kp1 = 3.0;
float kd1 = 0.6;
float ki1 = 1.4;
float kp2 = 10.0;
float kd2 = 0.5;
float ki2 = 1.0;

float joint_input1, joint_effort1, joint_setpoint1 = 0.0;
float joint_position1, joint_position2;
float joint_input2, joint_effort2, joint_setpoint2 = 0.0;
uint32_t sample_time_ms = 20;
float pid_rate;
char user_setpoint;

char in_buffer[100];
uint16_t char_idx = 0;

PID PID_Joint1(&joint_input1, &joint_effort1, &joint_setpoint1, kp1, ki1, kd1, sample_time_ms);
PID PID_Joint2(&joint_input2, &joint_effort2, &joint_setpoint2, kp2, ki2, kd2, sample_time_ms);

uint32_t millis()
{
    return to_ms_since_boot(get_absolute_time());
}

void initRobot()
{
    motor1.write(0.0);
    motor2.write(0.0);
    // PID PID_Joint1(&joint_input1, &joint_effort1, &joint_setpoint1, kp, ki, kd, sample_time_ms);
    PID_Joint1.set_output_limits(-90.0f, 90.0f);
    PID_Joint2.set_output_limits(-90.0f, 90.0f);
    joint_setpoint1 = 0;
    joint_setpoint2 = 0;
    pid_rate = float(sample_time_ms) / 1000.0f;
}

void updatePid(int32_t joint1_encoder_ticks, int32_t joint2_encoder_ticks)
{
    int32_t joint1_ticks = joint1_encoder_ticks;
    int32_t joint2_ticks = joint2_encoder_ticks;

    float motor1_vel = 0;
    float motor2_vel = 0;

    joint_position1 = float(joint1_ticks) * 360.0f / (80.0f * 127.7f * 4.0f);
    joint_position2 = float(joint2_ticks) * 360.0f / (80.0f * 65.5f * 3.4f);

    joint_input1 = joint_position1;
    joint_input2 = joint_position2;

    PID_Joint1.compute();
    PID_Joint2.compute();

    motor1_vel = (joint_effort1 / 90);
    motor2_vel = (joint_effort2 / 90);

    motor1.write(-motor1_vel);
    motor2.write(motor2_vel);
    // printf("Motor entregado %.2f \n", motor_vel);
}

bool timerCallback(repeating_timer_t *rt)
{
    updatePid(int32_t(encoder1.encoder_pos), int32_t(encoder2.encoder_pos));
    return true;
}

void encoders_callback(uint gpio, uint32_t events)
{
    encoder1.readPosition();
    encoder2.readPosition();
    encoder3.readPosition();
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

    int input_char;
    int input_char_index;
    char *char_pt1;
    char *char_pt2;
    char *char_pt3;

    float joint1_sp = 0.0;
    float joint2_sp = 0.0;

    while (true)
    {
        printf("Introduce el setpoint en formato joint1,joint2/\n");

        input_char = getchar_timeout_us(0); // Esperar la entrada del usuario
        // printf("%c \n", input_char);
        while (input_char != PICO_ERROR_TIMEOUT)
        {
            // printf(" %c ", ch);
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            putchar(input_char);                        // Print user input in console
            in_buffer[input_char_index++] = input_char; // Index user input to buffer array
            if (input_char == '/')
            {
                in_buffer[input_char_index] = 0; // end of string
                printf("\nreceived: %s\n", in_buffer);
                input_char_index = 0;
                joint1_sp = strtof(in_buffer, &char_pt1);    // Conversion string (char) to float
                joint2_sp = strtof(char_pt1 + 1, &char_pt2); // Add 1 to bring up the comma
                // printf("Position to joint1: %.2f and position to joint 2: %.2f \n", joint1_sp, joint2_sp);
                break;
            }
            joint_setpoint1 = joint1_sp;
            joint_setpoint2 = joint2_sp;
            input_char = getchar_timeout_us(0);
            printf("\n Caracter recibido \n");
        }
        // gpio_put(LED_PIN, false);
        // printf("Entradas recibidas");
        //printf("Effort: %.3f, %.3f \n", joint_effort1, joint_effort2);
        printf("Position1: %.3f\n", joint_position1);
        printf("Position2: %.3f\n \n", joint_position2);
        sleep_ms(500);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
    }
}