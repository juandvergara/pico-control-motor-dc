#include <cmath>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "dc_motor_v2.h"
#include "encoder.h"
#include "pid_controller.h"

#define I2C_PORT i2c0
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5
#define LED_PIN 25

#define SLIDEBASE_RELATION 0.008809710258418167f // 360.0f / (80.0f * 127.7f * 4.0f)
#define BASE_RELATION 0.007047768206734534f      // 360.0f / (80.0f * 127.7f * 5.0f)
#define SHOULDER_RELATION 0.008809710258418167f  // 360.0f / (80.0f * 127.7f * 4.0f)

static int SLAVE_ADDR = 0x15;

DCMotor slidebase_motor(M0_ENA_PIN, M0_ENB_PIN);
DCMotor base_motor(M1_ENA_PIN, M1_ENB_PIN);
DCMotor shoulder_motor(M2_ENA_PIN, M2_ENB_PIN);

Encoder slidebase_encoder(M0_ENC_A_PIN, M0_ENC_B_PIN);
Encoder base_encoder(M1_ENC_A_PIN, M1_ENC_B_PIN);
Encoder shoulder_encoder(M2_ENC_A_PIN, M2_ENC_B_PIN);

float kp1 = 0.05;
float kd1 = 0.0;
float ki1 = 0.0;
float kp2 = 0.05;
float kd2 = 0.0;
float ki2 = 0.0;

float slidebase_input, slidebase_effort, slidebase_setpoint = 0.0;
float base_input, base_effort, base_setpoint = 0.0;
float shoulder_input, shoulder_effort, shoulder_setpoint = 0.0;

float slidebase_position, base_position, shoulder_position;

uint32_t sample_time_ms = 20;
float pid_rate;

PID PID_slidebase(&slidebase_input, &slidebase_effort, &slidebase_setpoint, kp1, ki1, kd1, sample_time_ms);
PID PID_base(&base_input, &base_effort, &base_setpoint, kp2, ki2, kd2, sample_time_ms);
PID PID_shoulder(&shoulder_input, &shoulder_effort, &shoulder_setpoint, kp2, ki2, kd2, sample_time_ms);

uint32_t millis()
{
    return to_ms_since_boot(get_absolute_time());
}

void initRobot()
{
    slidebase_motor.write(0.0);
    base_motor.write(0.0);
    shoulder_motor.write(0.0);

    PID_slidebase.set_output_limits(-1.0f, 1.0f);
    PID_base.set_output_limits(-1.0f, 1.0f);
    PID_shoulder.set_output_limits(-1.0f, 1.0f);

    slidebase_setpoint = 0;
    base_setpoint = 0;
    shoulder_setpoint = 0;

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

    slidebase_position = float(joint1_ticks) * SLIDEBASE_RELATION;
    base_position = float(joint2_ticks) * BASE_RELATION;
    shoulder_position = float(joint3_ticks) * SHOULDER_RELATION;

    slidebase_input = slidebase_position;
    base_input = base_position;
    shoulder_input = shoulder_position;

    PID_slidebase.compute();
    PID_base.compute();
    PID_shoulder.compute();

    M0_ENC_INVERTED ? motor1_vel = -slidebase_effort : motor3_vel = shoulder_effort;
    M1_ENC_INVERTED ? motor2_vel = -base_effort : motor3_vel = shoulder_effort;
    M2_ENC_INVERTED ? motor3_vel = -shoulder_effort : motor3_vel = shoulder_effort;

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

    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

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

    uint8_t target_slave = 0;
    char in_buffer[100];
    int input_char;
    int input_char_index;
    char *char_pt1;
    char *char_pt2;
    char *char_pt3;
    char *char_pt4;
    char *char_pt5;

    float slidebase_sp = 0.0;
    float base_sp = 0.0;
    float shoulder_sp = 0.0;
    uint8_t elbow_sp = 0;
    uint8_t wrist_left_sp = 0;
    uint8_t wrist_right_sp = 0;

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
                slidebase_sp = strtof(in_buffer, &char_pt1); // Conversion string (char) to float
                base_sp = strtof(char_pt1 + 1, &char_pt2);
                shoulder_sp = strtof(char_pt2 + 1, &char_pt3); // Add 1 to bring up the comma
                elbow_sp = uint8_t(strtof(char_pt3 + 1, &char_pt4));
                wrist_left_sp = uint8_t(strtof(char_pt4 + 1, &char_pt5));
                wrist_right_sp = uint8_t(strtof(char_pt5 + 1, NULL));
                break;
            }
            slidebase_setpoint = slidebase_sp;
            base_setpoint = base_sp;
            shoulder_setpoint = shoulder_sp;
            input_char = getchar_timeout_us(0);
            printf("\n Caracter recibido \n");
        }
        i2c_write_blocking(I2C_PORT, SLAVE_ADDR, &elbow_sp, 1, false);
        i2c_write_blocking(I2C_PORT, SLAVE_ADDR, &wrist_left_sp, 1, false);
        i2c_write_blocking(I2C_PORT, SLAVE_ADDR, &wrist_right_sp, 1, false);
        printf("Slide base: sp %.3f, pos: %.3f, \n", slidebase_setpoint, slidebase_position);
        printf("Base: sp %.3f, pos: %.3f, \n", base_setpoint, base_position);
        printf("Shoulder: sp %.3f, pos: %.3f\n \n", shoulder_setpoint, shoulder_position);
        sleep_ms(500);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
    }
}