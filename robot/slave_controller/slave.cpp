#include <cmath>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "dc_motor_v2.h"
#include "encoder.h"
#include "pid_controller.h"

#define I2C_PORT i2c1
#define I2C_SDA_PIN 26
#define I2C_SCL_PIN 27
#define LED_PIN 25

#define ELBOW_RELATION 0.008809710258418167 // 360.0f / (80.0f * 127.7f * 4.0f)
#define WRIST_RELATION 0.049379770992366415 // 360.0f * 23.0f / (80.0f * 65.5f * 32.0f)

static int SLAVE_ADDR = 0x15;

DCMotor elbow_motor(M3_ENA_PIN, M3_ENB_PIN);
DCMotor wrist_left_motor(M4_ENA_PIN, M4_ENB_PIN);
DCMotor wrist_right_motor(M5_ENA_PIN, M5_ENB_PIN);

Encoder elbow_encoder(M3_ENC_A_PIN, M3_ENC_B_PIN);
Encoder wrist_left_encoder(M4_ENC_A_PIN, M4_ENC_B_PIN);
Encoder wrist_right_encoder(M5_ENC_A_PIN, M5_ENC_B_PIN);

float kp1 = 0.05;
float kd1 = 0.0;
float ki1 = 0.0;
float kp2 = 0.05;
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

    M3_ENC_INVERTED ? motor1_vel = -elbow_effort : motor1_vel = elbow_effort;
    M4_ENC_INVERTED ? motor2_vel = -wrist_left_effort : motor2_vel = wrist_left_effort;
    M5_ENC_INVERTED ? motor3_vel = -wrist_right_effort : motor3_vel = wrist_right_effort;

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

    uint8_t target_slave = 0;
    uint8_t elbow_sp = 0;
    uint8_t wrist_left_sp = 0;
    uint8_t wrist_right_sp = 0;

    while (true)
    {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        i2c_read_raw_blocking(I2C_PORT, &elbow_sp, 1);
        i2c_read_raw_blocking(I2C_PORT, &wrist_left_sp, 1);
        i2c_read_raw_blocking(I2C_PORT, &wrist_right_sp, 1);

        elbow_setpoint = elbow_sp;
        wrist_left_setpoint = wrist_left_sp;
        wrist_right_setpoint = -wrist_right_sp;
        // READ I2C INFO TO EACH JOINT

        printf("Elbow: sp %.3f, pos: %.3f, \n", elbow_setpoint, elbow_position);
        printf("Wrist left: sp %.3f, pos: %.3f, \n", wrist_left_setpoint, wrist_left_position);
        printf("Wrist right: sp %.3f, pos: %.3f\n \n", wrist_right_setpoint, wrist_right_position);
        sleep_ms(500);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
    }
}