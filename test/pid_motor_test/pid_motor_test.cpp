#include "pico/stdlib.h"
#include <cmath>
#include <stdio.h>
#include "dc_motor_v2.h"
#include "encoder.h"
#include "pid_filter.h"

Encoder encoder3(M3_ENC_A_PIN, M3_ENC_B_PIN);
Encoder encoder4(M4_ENC_A_PIN, M4_ENC_B_PIN);
Encoder encoder5(M5_ENC_A_PIN, M5_ENC_B_PIN);

DCMotor motor3(M3_ENA_PIN, M3_ENB_PIN);
DCMotor motor4(M4_ENA_PIN, M4_ENB_PIN);
DCMotor motor5(M5_ENA_PIN, M5_ENB_PIN);

float kp1 = 0.060;
float kd1 = 0.0007;
float ki1 = 0.0060;
float kp2 = 0.020;
float kd2 = 0.0001;
float ki2 = 0.0015;

float joint_input1, joint_effort1, joint_setpoint1 = 0.0;
float joint_position1, joint_position2, joint_position3;
float joint_input2, joint_effort2, joint_setpoint2 = 0.0;
float joint_input3, joint_effort3, joint_setpoint3 = 0.0;

uint32_t sample_time_ms = 20;
float pid_rate;

char in_buffer[500];
uint16_t char_idx = 0;

PID PID_Joint1(&joint_input1, &joint_effort1, &joint_setpoint1, kp1, ki1, kd1, sample_time_ms);
PID PID_Joint2(&joint_input2, &joint_effort2, &joint_setpoint2, kp2, ki2, kd2, sample_time_ms);
PID PID_Joint3(&joint_input3, &joint_effort3, &joint_setpoint3, kp2, ki2, kd2, sample_time_ms);

uint32_t millis()
{
    return to_ms_since_boot(get_absolute_time());
}

void initRobot()
{
    motor3.write(0.0);
    motor4.write(0.0);
    motor5.write(0.0);
    // PID PID_Joint1(&joint_input1, &joint_effort1, &joint_setpoint1, kp, ki, kd, sample_time_ms);
    PID_Joint1.set_output_limits(-1.0f, 1.0f);
    PID_Joint2.set_output_limits(-1.0f, 1.0f);
    PID_Joint3.set_output_limits(-1.0f, 1.0f);
    joint_setpoint1 = 0;
    joint_setpoint2 = 0;
    joint_setpoint3 = 0;
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

    joint_position1 = float(joint1_ticks) * 360.0f / (80.0f * 127.7f * 4.0f);
    joint_position2 = float(joint2_ticks) * 360.0f * 23.0f / (80.0f * 65.5f * 32.0f);
    joint_position3 = float(joint3_ticks) * 360.0f * 23.0f / (80.0f * 65.5f * 32.0f);

    joint_input1 = joint_position1;
    joint_input2 = joint_position2;
    joint_input3 = joint_position3;

    PID_Joint1.compute();
    PID_Joint2.compute();
    PID_Joint3.compute();

    motor1_vel = joint_effort1;
    motor2_vel = joint_effort2;
    motor3_vel = joint_effort3;

    motor3.write(-motor1_vel);
    motor4.write(motor2_vel);
    motor5.write(-motor3_vel);
    // printf("Motor entregado %.2f \n", motor_vel);
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
        // printf("Introduce el setpoint en formato joint1,joint2/\n");

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
                //printf("\nreceived: %s\n", in_buffer);
                input_char_index = 0;
                joint1_sp = strtof(in_buffer, &char_pt1);    // Conversion string (char) to float
                joint2_sp = strtof(char_pt1 + 1, &char_pt2); // Conversion string (char) to float
                joint3_sp = strtof(char_pt2 + 1, &char_pt3); // Add 1 to bring up the comma
                // printf("Position to joint1: %.2f and position to joint 2: %.2f \n", joint1_sp, joint2_sp);
                break;
            }
            joint_setpoint1 = joint1_sp;
            joint_setpoint2 = joint2_sp;
            joint_setpoint3 = -joint3_sp;
            input_char = getchar_timeout_us(0);
            //printf("\n Caracter recibido \n");
        }
        // gpio_put(LED_PIN, false);
        // printf("Entradas recibidas");
        //printf("Effort: J1: %.3f, J2: %.3f, J3: %.3f \n", joint_effort1, joint_effort2, joint_effort3);
        //printf("Position: J1: %.3f, J2: %.3f \n", joint_position1, joint_position2);
        //printf("Ef1: %.4f, \n", joint_effort1);
        printf("Pos1: %.4f\n \n", joint_position1);/*/
        printf("Ef2: %.4f, \n", joint_effort2);
        printf("Pos2: %.4f\n \n", joint_position2);*/
        sleep_ms(200);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
    }
}