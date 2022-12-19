#include <cmath>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "dc_motor_v2.h"
#include "encoder.h"
#include "pid_filter.h"

#define I2C_PORT i2c1
#define I2C_SDA_PIN 26
#define I2C_SCL_PIN 27
#define LED_PIN 25

#define SHOULDER_RELATION 0.008809710258418167f // 360.0f / (80.0f * 127.7f * 4.0f)
#define ELBOW_RELATION 0.008809710258418167f    // 360.0f / (80.0f * 127.7f * 4.0f)
#define WRIST_RELATION 0.038643194504079006f    // 0.03435114503816794f     // 0.03864325091758399f 360.0f / (80.0f * 65.5f * 2.0f)

#define PRINT_STATE 'e'
#define COMMAND_POS 'p'
#define COMMAND_VEL 'v'
#define SET_VEL_MODE 's'
#define UNSET_VEL_MODE 'n'

static int SLAVE_ADDR = 0x15;

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

DCMotor elbow_motor(M3_ENA_PIN, M3_ENB_PIN);
DCMotor wrist_left_motor(M4_ENA_PIN, M4_ENB_PIN);
DCMotor wrist_right_motor(M5_ENA_PIN, M5_ENB_PIN);

Encoder elbow_encoder(M3_ENC_A_PIN, M3_ENC_B_PIN);
Encoder wrist_left_encoder(M4_ENC_A_PIN, M4_ENC_B_PIN);
Encoder wrist_right_encoder(M5_ENC_A_PIN, M5_ENC_B_PIN);

float kp1 = 0.2;
float ki1 = 0.00008;
float kd1 = 0.00007;
float kp2 = 0.2;
float ki2 = 0.00008;
float kd2 = 0.00007;

float v1Prev = 0.0;
float v2Prev = 0.0;
float v3Prev = 0.0;

float slidebase_status = 0;
float base_status = 0;
float shoulder_status = 0;
float elbow_sp = 0;
float wrist_left_sp = 0;
float wrist_right_sp = 0;

uint32_t sample_time_ms = 10;
float pid_rate;

PID PID_elbow(&elbow_joint.position, &elbow_joint.velocity, &elbow_joint.effort, &elbow_joint.ref_position, &elbow_joint.ref_velocity,
              kp1, ki1, kd1, sample_time_ms);
PID PID_wrist_left(&wrist_left_joint.position, &wrist_left_joint.velocity, &wrist_left_joint.effort, &wrist_left_joint.ref_position, &wrist_left_joint.ref_velocity,
                   kp2, ki2, kd2, sample_time_ms);
PID PID_wrist_right(&wrist_right_joint.position, &wrist_right_joint.velocity, &wrist_right_joint.effort, &wrist_right_joint.ref_position, &wrist_right_joint.ref_velocity,
                    kp2, ki2, kd2, sample_time_ms);

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

    float position_elbow = float(joint1_ticks) * ELBOW_RELATION;
    float position_wrist_left = float(joint2_ticks) * WRIST_RELATION;
    float position_wrist_right = float(joint3_ticks) * WRIST_RELATION;

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

    PID_elbow.compute();
    PID_wrist_left.compute();
    PID_wrist_right.compute();

    M3_ENC_INVERTED ? motor1_vel = -elbow_joint.effort : motor1_vel = elbow_joint.effort;
    M4_ENC_INVERTED ? motor2_vel = -wrist_left_joint.effort : motor2_vel = wrist_left_joint.effort;
    M5_ENC_INVERTED ? motor3_vel = -wrist_right_joint.effort : motor3_vel = wrist_right_joint.effort;

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

// void set_vel_mode(bool mode)
// {
//     if (mode)
//     {
//         PID_elbow.set_gains(0.0, 0.0, kd1);
//         PID_wrist_left.set_gains(0.0, 0.0, kd1);
//         PID_wrist_right.set_gains(0.0, 0.0, kd1);
//     }
//     else
//     {
//         elbow_joint.ref_position = elbow_joint.position;
//         wrist_left_joint.ref_position = wrist_left_joint.position;
//         wrist_right_joint.ref_position = wrist_right_joint.position;
//         PID_elbow.set_gains(kp1, ki1, kd1);
//         PID_wrist_left.set_gains(kp1, ki1, kd1);
//         PID_wrist_right.set_gains(kp1, ki1, kd1);
//     }

// }

void print_state()
{
    printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f \n", slidebase_status, -base_status, shoulder_status,
           -(elbow_joint.position + shoulder_status),
           (wrist_right_joint.position - wrist_left_joint.position)/2.0 - shoulder_status
           + (elbow_joint.position + shoulder_status),
           (wrist_left_joint.position + wrist_right_joint.position)/2.0);

    /*printf("Slide base: pos: %.3f, \n", slidebase_status);
    printf("Base: pos: %.3f, \n", base_status);
    printf("Shoulder: pos: %.3f \n", shoulder_status);
    printf("Elbow: sp %.3f, pos: %.3f, \n", elbow_setpoint, elbow_position);
    printf("Wrist left: sp %.3f, pos: %.3f, \n", wrist_left_setpoint, wrist_left_position);
    printf("Wrist right: sp %.3f, pos: %.3f\n \n", wrist_right_setpoint, wrist_right_position);*/
}

void process_command(uint8_t cmd)
{
    switch (cmd)
    {
    case (COMMAND_POS):
        elbow_joint.ref_position = -round(elbow_sp / ELBOW_RELATION) * ELBOW_RELATION;
        wrist_left_joint.ref_position = round(wrist_left_sp / WRIST_RELATION) * WRIST_RELATION + elbow_joint.ref_position;
        wrist_right_joint.ref_position = round(wrist_right_sp / WRIST_RELATION) * WRIST_RELATION - elbow_joint.ref_position;
        break;

    case (COMMAND_VEL):
        elbow_joint.ref_velocity = -round(elbow_sp / ELBOW_RELATION) * ELBOW_RELATION;
        wrist_left_joint.ref_velocity = round(wrist_left_sp / WRIST_RELATION) * WRIST_RELATION + elbow_joint.ref_velocity;
        wrist_right_joint.ref_velocity = round(wrist_right_sp / WRIST_RELATION) * WRIST_RELATION + elbow_joint.ref_velocity;
        break;
    case (PRINT_STATE):
        print_state();
        break;
        // case (SET_VEL_MODE):
        //     set_vel_mode(true);
        //     break;
        // case (UNSET_VEL_MODE):
        //     set_vel_mode(false);
        //     break;
    }
}

int main()
{
    stdio_init_all();
    printf("Slave control");

    i2c_init(I2C_PORT, 100 * 1000);
    i2c_set_slave_mode(I2C_PORT, true, SLAVE_ADDR);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

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

    uint8_t target_elbow[4] = {0, 0, 0, 0};
    uint8_t target_wrist_left[4] = {0, 0, 0, 0};
    uint8_t target_wrist_right[4] = {0, 0, 0, 0};
    uint8_t status_slidebase[4] = {0, 0, 0, 0};
    uint8_t status_base[4] = {0, 0, 0, 0};
    uint8_t status_shoulder[4] = {0, 0, 0, 0};

    uint8_t command;

    while (true)
    {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);

        i2c_read_raw_blocking(I2C_PORT, &command, 1);

        i2c_read_raw_blocking(I2C_PORT, target_elbow, 4);
        i2c_read_raw_blocking(I2C_PORT, target_wrist_left, 4);
        i2c_read_raw_blocking(I2C_PORT, target_wrist_right, 4);
        i2c_read_raw_blocking(I2C_PORT, status_slidebase, 4);
        i2c_read_raw_blocking(I2C_PORT, status_base, 4);
        i2c_read_raw_blocking(I2C_PORT, status_shoulder, 4);

        slidebase_status = *(float *)&status_slidebase;
        base_status = *(float *)&status_base;
        shoulder_status = *(float *)&status_shoulder;
        elbow_sp = *(float *)&target_elbow + shoulder_status;
        // wrist_left_sp = *(float *)&target_wrist_left;
        // wrist_right_sp = *(float *)&target_wrist_right;

        wrist_left_sp = *(float *)&target_wrist_right - *(float *)&target_wrist_left;
        wrist_right_sp = *(float *)&target_wrist_right + *(float *)&target_wrist_left;

        process_command(command);

        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(10);
    }
}