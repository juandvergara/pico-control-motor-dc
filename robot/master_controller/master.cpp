#include <cmath>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "dc_motor_v2.h"
#include "encoder.h"
#include "pid_filter.h"

#define I2C_PORT i2c0
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5
#define LED_PIN 25

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
    char in_buffer[500];
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
    float elbow_sp = 0;
    float wrist_left_sp = 0;
    float wrist_right_sp = 0;

    while (true)
    {
        input_char = getchar_timeout_us(0);
        while (input_char != PICO_ERROR_TIMEOUT)
        {
            gpio_put(PICO_DEFAULT_LED_PIN, 1);          // Print user input in console
            putchar(input_char);
            in_buffer[input_char_index++] = input_char; // Index user input to buffer array
            if (input_char == '/')
            {
                in_buffer[input_char_index] = 0;
                input_char_index = 0;
                slidebase_sp = strtof(in_buffer, &char_pt1); // Conversion string (char) to float
                base_sp = strtof(char_pt1 + 1, &char_pt2);
                shoulder_sp = strtof(char_pt2 + 1, &char_pt3); // Add 1 to bring up the comma
                elbow_sp = strtof(char_pt3 + 1, &char_pt4);
                wrist_left_sp = strtof(char_pt4 + 1, &char_pt5);
                wrist_right_sp = strtof(char_pt5 + 1, NULL);
                break;
            }
            slidebase_joint.ref_position = slidebase_sp;
            base_joint.ref_position = round(base_sp / BASE_RELATION) * BASE_RELATION;
            shoulder_joint.ref_position = round(shoulder_sp / SHOULDER_RELATION) * SHOULDER_RELATION;
            input_char = getchar_timeout_us(0);
        }

        uint8_t *target_slave1;
        target_slave1 = (uint8_t *)(&elbow_sp);
        uint8_t *target_slave2;
        target_slave2 = (uint8_t *)(&wrist_left_sp);
        uint8_t *target_slave3;
        target_slave3 = (uint8_t *)(&wrist_right_sp);

        uint8_t *status_slidebase;
        status_slidebase = (uint8_t *)(&slidebase_joint.position);
        uint8_t *status_base;
        status_base = (uint8_t *)(&base_joint.position);
        uint8_t *status_shoulder;
        status_shoulder = (uint8_t *)(&shoulder_joint.position);

        i2c_write_blocking(I2C_PORT, SLAVE_ADDR, target_slave1, 4, false);
        i2c_write_blocking(I2C_PORT, SLAVE_ADDR, target_slave2, 4, false);
        i2c_write_blocking(I2C_PORT, SLAVE_ADDR, target_slave3, 4, false);
        i2c_write_blocking(I2C_PORT, SLAVE_ADDR, status_slidebase, 4, false);
        i2c_write_blocking(I2C_PORT, SLAVE_ADDR, status_base, 4, false);
        i2c_write_blocking(I2C_PORT, SLAVE_ADDR, status_shoulder, 4, false);

        /*printf("Slide base: sp %.3f, pos: %.3f, \n", slidebase_joint.ref_position, slidebase_joint.position);
        printf("Base: sp %.3f, pos: %.3f, \n", base_joint.ref_position, base_joint.position);
        printf("Shoulder: sp %.3f, pos: %.3f\n \n", shoulder_joint.ref_position, shoulder_joint.position);*/
        sleep_ms(20);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
    }
}