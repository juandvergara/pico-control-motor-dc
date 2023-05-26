#include <cmath>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "dc_motor_v2.h"
#include "encoder.h"
#include "pid_v3.h"
#include "pid_controller.h"
#include "command.h"

#define MASTER
#include "pins.h"

#define SLIDEBASE_RELATION 0.000361261652855874f // 1.0f / (1.3516f * 512.0f * 4.0f)
#define BASE_RELATION 0.007047768206734534f      // 360.0f / (80.0f * 127.7f * 5.0f)
#define SHOULDER_RELATION 0.008809710258418167f  // 360.0f / (80.0f * 127.7f * 4.0f)

#define HOME_BASE_ANGLE 87.5f
#define HOME_SHOULDER_ANGLE 118.5f

#define DEG_S 8.0f

#define TOP_TEMP 4095
#define CONTROL_PIN 5
#define ADC_PIN 26
#define ADC_CHANNEL 0

struct Joint
{
    float position = 0, velocity = 0, effort = 0;
    float ref_position = 0, ref_velocity = 0;
};

struct Nozzle
{
    float target_temperture, actual_temperture, output_temperture, previous_temperature;
};

Nozzle hotend;

uint32_t temp_sample_time_ms = 500;
uint _slice_num;
uint _channel;

float kp_temp = 0.03324 / 2.0;
float ki_temp = 0.000538 / 2.0; // 0.0538
float kd_temp = 0.07552 / 2.0;  // 0.7552;

PID PID_hotend(&hotend.actual_temperture, &hotend.output_temperture, &hotend.target_temperture, 
                kp_temp, ki_temp, kd_temp, temp_sample_time_ms);

Joint slidebase_joint, base_joint, shoulder_joint;
DCMotor slidebase_motor{M0_ENA_PIN, M0_ENB_PIN}, base_motor{M1_ENA_PIN, M1_ENB_PIN}, shoulder_motor{M2_ENA_PIN, M2_ENB_PIN};
Encoder slidebase_encoder{M0_ENC_A_PIN, M0_ENC_B_PIN}, base_encoder{M1_ENC_A_PIN, M1_ENC_B_PIN}, shoulder_encoder{M2_ENC_A_PIN, M2_ENC_B_PIN};

float kp = 0.510;
float ki = 1.015;
float kd = 0.016;

float k_h = 0.01;
float k_gamma = 1.2;

float v1Prev = 0.0;
float v2Prev = 0.0;
float v3Prev = 0.0;

uint32_t sample_time_ms = 5;
float pid_rate = float(sample_time_ms) / 1000.0f;

PID_V3 PID_slidebase(&slidebase_joint.position, &slidebase_joint.velocity, &slidebase_joint.effort, &slidebase_joint.ref_position, &slidebase_joint.ref_velocity,
                     kp, ki, kd, sample_time_ms, k_h, k_gamma),
    PID_base(&base_joint.position, &base_joint.velocity, &base_joint.effort, &base_joint.ref_position, &base_joint.ref_velocity,
             kp, ki, kd, sample_time_ms, k_h, k_gamma),
    PID_shoulder(&shoulder_joint.position, &shoulder_joint.velocity, &shoulder_joint.effort, &shoulder_joint.ref_position, &shoulder_joint.ref_velocity,
                 kp, ki, kd, sample_time_ms, k_h, k_gamma);

uint32_t millis() { return to_ms_since_boot(get_absolute_time()); }

/*TEMPERTURE CONTROL*/
void temp_init()
{
    _slice_num = pwm_gpio_to_slice_num(CONTROL_PIN);
    _channel = pwm_gpio_to_channel(CONTROL_PIN);
    gpio_set_function(CONTROL_PIN, GPIO_FUNC_PWM);
    pwm_set_wrap(_slice_num, TOP_TEMP);
    pwm_set_chan_level(_slice_num, _channel, 0);

    pwm_set_enabled(_slice_num, true);

    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(ADC_CHANNEL);

    PID_hotend.set_output_limits(0.0, 1.0);
}

void temp_write(float duty_cycle)
{
    if (duty_cycle > 1.0f)
        duty_cycle = 1.0f;
    if (duty_cycle < -1.0f)
        duty_cycle = -1.0f;
    pwm_set_chan_level(_slice_num, _channel, (int16_t)(duty_cycle * TOP_TEMP));
    pwm_set_enabled(_slice_num, true);
}

void temp_calculate()
{
    float temp = -20;

    const float LUT[][2] = {
        {92, 300}, {100, 295}, {108, 290}, {112, 285}, {124, 280}, {132, 275}, {140, 270}, {152, 265}, {164, 260}, {176, 255}, {192, 250}, {208, 245}, {224, 240}, {244, 235}, {264, 230}, {284, 225}, {312, 220}, {336, 215}, {368, 210}, {400, 205}, {436, 200}, {480, 195}, {524, 190}, {572, 185}, {624, 180}, {685, 175}, {749, 170}, {821, 165}, {897, 160}, {981, 155}, {1073, 150}, {1173, 145}, {1281, 140}, {1393, 135}, {1517, 130}, {1645, 125}, {1781, 120}, {1921, 115}, {2066, 110}, {2214, 105}, {2366, 100}, {2514, 95}, {2662, 90}, {2810, 85}, {2950, 80}, {3082, 75}, {3206, 70}, {3322, 65}, {3431, 60}, {3527, 55}, {3615, 50}, {3691, 45}, {3759, 40}, {3819, 35}, {3867, 30}, {3911, 25}, {3943, 20}, {3975, 15}, {3999, 10}, {4019, 5}, {4035, 0}, {4051, -5}, {4067, -10}, {4083, -15}};

    uint16_t raw_read = adc_read();

    for (size_t i = 0; i < 64; i++)
    {
        if (raw_read >= LUT[i][0] && raw_read <= LUT[i + 1][0])
        {
            temp = ((LUT[i + 1][1] - LUT[i][1]) / (LUT[i + 1][0] - LUT[i][0])) * (raw_read - LUT[i + 1][0]) + LUT[i + 1][1];
        }
    }

    hotend.actual_temperture = 0.854 * hotend.actual_temperture + 0.0728 * temp + 0.0728 * hotend.previous_temperature;
    hotend.previous_temperature = temp;

    PID_hotend.compute();
    temp_write(hotend.output_temperture);
}

bool temp_Callback(repeating_timer_t *rt)
{
    temp_calculate();
    return true;
}

void set_hotend_temp(float target)
{
    hotend.target_temperture = target;
}

/*TEMPERTURE CONTROL*/
void initRobot()
{
    slidebase_motor.write(0.0);
    base_motor.write(0.0);
    shoulder_motor.write(0.0);

    PID_slidebase.set_output_limits(-1.0f, 1.0f);
    PID_base.set_output_limits(-1.0f, 1.0f);
    PID_shoulder.set_output_limits(-1.0f, 1.0f);

    gpio_init(M0_HOME_SW);
    gpio_pull_up(M0_HOME_SW);
    gpio_init(M1_HOME_SW);
    gpio_pull_up(M1_HOME_SW);
    gpio_init(M2_HOME_SW);
    gpio_pull_up(M2_HOME_SW);
}

void set_vel_mode(float mode, bool print_msg)
{
    if (mode == 1.0)
    {
        if (print_msg)
            printf("Velocity control mode on \n");
        PID_slidebase.set_gains(0.0, 0.0, kd, k_h, k_gamma);
        PID_base.set_gains(0.0, 0.0, kd, k_h, k_gamma);
        PID_shoulder.set_gains(0.0, 0.0, kd, k_h, k_gamma);
    }
    else if (mode == 0.0)
    {
        if (print_msg)
            printf("Velocity control mode off \n");
        slidebase_joint.ref_position = slidebase_joint.position;
        base_joint.ref_position = base_joint.position;
        shoulder_joint.ref_position = shoulder_joint.position;
        PID_slidebase.set_gains(kp, ki, kd, k_h, k_gamma);
        PID_base.set_gains(kp, ki, kd, k_h, k_gamma);
        PID_shoulder.set_gains(kp, ki, kd, k_h, k_gamma);
    }
    else
    {
        if (print_msg)
            printf("Wrong command! Set 1 to vel mode, 0 to pos mode \n");
    }
}

bool home_body()
{
    bool base_home = false;
    float initial_time, current_time;

    while (!base_home)
    {
        float theta0 = base_joint.position, dot_theta0 = 0, thetaf, dot_thetaf = 0, t_final;
        float a, b, c, d;

        printf("Fixing body home \n");

        thetaf = gpio_get(M1_HOME_SW) ? 90 : -20;

        t_final = abs(thetaf - theta0) / DEG_S;
        a = theta0;
        b = dot_theta0;
        c = 3.0 * (thetaf - theta0) / pow(t_final, 2) - 2 * dot_theta0 / pow(t_final, 2) - dot_thetaf / pow(t_final, 2);
        d = -2.0 * (thetaf - theta0) / pow(t_final, 3) + (dot_thetaf - dot_theta0) / pow(t_final, 2);

        initial_time = millis() / 1e3;
        current_time = initial_time;

        while ((current_time - initial_time) < t_final)
        {
            float time_process = current_time - initial_time;
            base_joint.ref_position = a + b * time_process + c * pow(time_process, 2) + d * pow(time_process, 3);
            base_joint.ref_velocity = b + 2 * c * time_process + 3 * d * pow(time_process, 2);
            if (!gpio_get(M1_HOME_SW))
            {
                base_encoder.encoder_pos = 0;
                base_joint.ref_position = 0;
                base_joint.ref_velocity = 0;
                base_home = true;
                break;
            }
            current_time = millis() / 1e3;
        }
    }
    return base_home;
}

bool home_shoulder()
{
    bool shoulder_home = false;
    float initial_time, current_time;

    while (!shoulder_home)
    {
        float theta0 = shoulder_joint.position, dot_theta0 = 0, thetaf, dot_thetaf = 0, t_final;
        float a, b, c, d;

        printf("Fixing shoulder home \n");

        thetaf = gpio_get(M2_HOME_SW) ? 180 : -10;

        t_final = abs(thetaf - theta0) / DEG_S;
        a = theta0;
        b = dot_theta0;
        c = 3.0 * (thetaf - theta0) / pow(t_final, 2) - 2 * dot_theta0 / pow(t_final, 2) - dot_thetaf / pow(t_final, 2);
        d = -2.0 * (thetaf - theta0) / pow(t_final, 3) + (dot_thetaf - dot_theta0) / pow(t_final, 2);

        initial_time = millis() / 1e3;
        current_time = initial_time;

        while ((current_time - initial_time) < t_final)
        {
            float time_process = current_time - initial_time;
            shoulder_joint.ref_position = a + b * time_process + c * pow(time_process, 2) + d * pow(time_process, 3);
            shoulder_joint.ref_velocity = b + 2 * c * time_process + 3 * d * pow(time_process, 2);
            if (!gpio_get(M2_HOME_SW))
            {
                shoulder_encoder.encoder_pos = 0;
                shoulder_joint.ref_position = 0;
                shoulder_joint.ref_velocity = 0;
                shoulder_home = true;
                break;
            }
            current_time = millis() / 1e3;
        }
    }
    return shoulder_home;
}

void print_state_joints()
{
    printf("%.3f,%.3f,%.3f\n",
           slidebase_joint.position, base_joint.position, shoulder_joint.position);
}

void print_vel_joints()
{
    printf("%.3f,%.3f,%.3f",
           slidebase_joint.velocity, base_joint.velocity, shoulder_joint.velocity);
}

void command_callback(char *buffer)
{
    char *current;
    char *previous;
    char *token = strtok(buffer, " ");
    char command = *token;

    float result[50];

    switch (command)
    {
    case (COMMAND_POS):
        token = strtok(NULL, " ");

        result[0] = strtof(token, &previous);

        for (int i = 0; i < 2; i++)
        {
            result[i + 1] = strtof(previous + 1, &current);
            previous = current;
        }
        slidebase_joint.ref_position = round(result[0] / SLIDEBASE_RELATION) * SLIDEBASE_RELATION;
        base_joint.ref_position = round(result[1] / BASE_RELATION) * BASE_RELATION;
        shoulder_joint.ref_position = round(result[2] / SHOULDER_RELATION) * SHOULDER_RELATION;
        break;

    case (COMMAND_VEL):
        token = strtok(NULL, " ");

        result[0] = strtof(token, &previous);
        printf("%.1f", result[0]);

        for (int i = 0; i < 2 - 1; i++)
        {
            result[i + 1] = strtof(previous + 1, &current);
            previous = current;
            printf(", %.1f", result[i + 1]);
        }
        printf("\n");
        slidebase_joint.ref_velocity = result[0];
        base_joint.ref_velocity = result[1];
        shoulder_joint.ref_velocity = result[2];
        break;
    case (READ_ENCODER):
        print_state_joints();
        break;
    case (READ_VEL_ENCODER):
        print_vel_joints();
        break;
    case (SET_HOTEND_TEMPERATURE):
        token = strtok(NULL, " ");
        float target;
        target = strtof(token, &previous);
        set_hotend_temp(target);
        break;
    case (SET_VEL_MODE):
        token = strtok(NULL, " ");
        float mode;
        mode = strtof(token, &previous);
        set_vel_mode(mode, true);
        break;
    case (HOME):
        home_shoulder();
        home_body();
        slidebase_encoder.encoder_pos = 0;
        slidebase_joint.ref_position = 0;
        base_encoder.encoder_pos = round(HOME_BASE_ANGLE / BASE_RELATION);
        base_joint.ref_position = round(HOME_BASE_ANGLE / BASE_RELATION) * BASE_RELATION;
        shoulder_encoder.encoder_pos = round(HOME_SHOULDER_ANGLE / SHOULDER_RELATION);
        shoulder_joint.ref_position = round(HOME_SHOULDER_ANGLE / SHOULDER_RELATION) * SHOULDER_RELATION;
        break;
    case (CLEAR_JOINTS):
        printf("Encoder variables cleaned! \n");
        slidebase_encoder.encoder_pos = slidebase_joint.ref_position = 0;
        base_encoder.encoder_pos = base_joint.ref_position = 0;
        shoulder_encoder.encoder_pos = shoulder_joint.ref_position = 0;
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
            command_callback(in_buffer);
            break;
        }
        input_std = getchar_timeout_us(0);
    }
    // gpio_put(PICO_DEFAULT_LED_PIN, 0);
}

void updatePid(int32_t joint1_encoder_ticks, int32_t joint2_encoder_ticks, int32_t joint3_encoder_ticks)
{
    float position_slidebase = float(joint1_encoder_ticks) * SLIDEBASE_RELATION;
    float position_base = float(joint2_encoder_ticks) * BASE_RELATION;
    float position_shoulder = float(joint3_encoder_ticks) * SHOULDER_RELATION;

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

    slidebase_motor.write(M0_ENC_INVERTED ? -slidebase_joint.effort : slidebase_joint.effort);
    base_motor.write(M1_ENC_INVERTED ? -base_joint.effort : base_joint.effort);
    shoulder_motor.write(M2_ENC_INVERTED ? -shoulder_joint.effort : shoulder_joint.effort);
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

void core1_comm()
{
    int input_char;

    while (true)
    {
        input_char = getchar_timeout_us(0);
        process_user_input(input_char);
    }
}

int main()
{
    stdio_init_all();
    printf("Master control");

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    initRobot();
    temp_init();

    multicore_launch_core1(core1_comm);
    sleep_ms(500);

    gpio_set_irq_enabled_with_callback(M0_ENC_A_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoders_callback);
    gpio_set_irq_enabled_with_callback(M1_ENC_A_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoders_callback);
    gpio_set_irq_enabled_with_callback(M2_ENC_A_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoders_callback);

    repeating_timer_t timer;
    if (!add_repeating_timer_ms(-sample_time_ms, timerCallback, NULL, &timer))
    {
        printf("Failure by not set timer!! \n");
    }

    sleep_ms(200);
    repeating_timer_t timer_temp;
    if (!add_repeating_timer_ms(-temp_sample_time_ms, temp_Callback, NULL, &timer_temp))
    {
        printf("Failure by not set timer!! \n");
    }

    while (true)
    {
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(250);
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(250);
        // printf("Actual temp %.3f, %.3f\n", hotend.actual_temperture, hotend.target_temperture);
    }
}
