#include <cmath>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/i2c.h"
#include "dc_motor_v2.h"
#include "encoder.h"
#include "pid_filter.h"
#include "command.h"

#define SLAVE
#include "pins.h"

#define STEP_PIN 0
#define DIR_PIN 1

#define SHOULDER_RELATION 0.008809710258418167f // 360.0f / (80.0f * 127.7f * 4.0f)
#define ELBOW_RELATION 0.008809710258418167f    // 360.0f / (80.0f * 127.7f * 4.0f)
#define WRIST_RELATION 0.038643194504079006f    // 0.03435114503816794f     // 0.03864325091758399f 360.0f / (80.0f * 65.5f * 2.0f)

#define HOME_ELBOW_ANGLE -5.0f
#define HOME_WRIST_LEFT_ANGLE -20.0f
#define HOME_WRIST_RIGHT_ANGLE -110.0f

#define DEG_S 8.0f

struct Joint
{
    float position = 0, velocity = 0, effort = 0;
    float ref_position = 0, ref_velocity = 0;
};

Joint elbow_joint, wrist_left_joint, wrist_right_joint;
DCMotor elbow_motor(M3_ENA_PIN, M3_ENB_PIN), wrist_left_motor(M4_ENA_PIN, M4_ENB_PIN), wrist_right_motor(M5_ENA_PIN, M5_ENB_PIN);
Encoder elbow_encoder(M3_ENC_A_PIN, M3_ENC_B_PIN), wrist_left_encoder(M4_ENC_A_PIN, M4_ENC_B_PIN), wrist_right_encoder(M5_ENC_A_PIN, M5_ENC_B_PIN);

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
float pid_rate = float(sample_time_ms) / 1000.0f;

PID_V2 PID_elbow(&elbow_joint.position, &elbow_joint.velocity, &elbow_joint.effort, &elbow_joint.ref_position, &elbow_joint.ref_velocity,
                 kp1, ki1, kd1, sample_time_ms),
    PID_wrist_left(&wrist_left_joint.position, &wrist_left_joint.velocity, &wrist_left_joint.effort, &wrist_left_joint.ref_position, &wrist_left_joint.ref_velocity,
                   kp2, ki2, kd2, sample_time_ms),
    PID_wrist_right(&wrist_right_joint.position, &wrist_right_joint.velocity, &wrist_right_joint.effort, &wrist_right_joint.ref_position, &wrist_right_joint.ref_velocity,
                    kp2, ki2, kd2, sample_time_ms);

/*STEPPER FUNC*/

// const uint STEPS_PER_REVOLUTION = 200;
// const uint MICROSTEPS = 32;
// const uint MS_DELAY = 1;

// Definición de constantes
#define ANGLE_PER_STEP 1.8 // Ángulo que se desplaza el motor por cada paso
#define STEPS_PER_REVOLUTION 200 // Número de pasos que da una revolución completa
#define MICROSECONDS_PER_MICROSTEP 31.25

float stepper_deg, stepper_speed;

// void step(bool dir)
// {
//     gpio_put(DIR_PIN, dir);
//     gpio_put(STEP_PIN, 1);
//     sleep_us(1);
//     gpio_put(STEP_PIN, 0);
//     sleep_ms(MS_DELAY);
// }

// void rotateDegrees(float degrees, float speed)
// {
//     int steps = (int)round(degrees * (STEPS_PER_REVOLUTION / 360.0) * MICROSTEPS);
//     float delay = 1.0 / speed;
//     for (int i = 0; i < steps; i++)
//     {
//         step(0);
//         sleep_ms(delay * 1000);
//     }
// }

void moveSteps(int steps, bool direction, int speed) {
  gpio_put(DIR_PIN, direction);
  uint32_t delay_us = speed / 2; // Cálculo del retardo en microsegundos (la mitad del tiempo que está activo el pulso)
  for (int i = 0; i < abs(steps); i++) {
    gpio_put(STEP_PIN, 1);
    sleep_us(delay_us);
    gpio_put(STEP_PIN, 0);
    sleep_us(delay_us);
  }
}

void moveDegrees(float degrees, int speed) {
  float steps = degrees / ANGLE_PER_STEP * 32;
  bool direction = (degrees > 0);
  int microseconds_per_step = 1000000 / (speed * 32); // Cálculo de la duración de cada paso completo
  moveSteps(steps, direction, microseconds_per_step);
}

/*STEPPER FUNC*/

uint32_t millis() { return to_ms_since_boot(get_absolute_time()); }

void initRobot()
{
    elbow_motor.write(0.0);
    wrist_left_motor.write(0.0);
    wrist_right_motor.write(0.0);

    PID_elbow.set_output_limits(-1.0f, 1.0f);
    PID_wrist_left.set_output_limits(-1.0f, 1.0f);
    PID_wrist_right.set_output_limits(-1.0f, 1.0f);

    gpio_init(M3_HOME_SW);
    gpio_pull_up(M3_HOME_SW);
    gpio_init(M4_HOME_SW);
    gpio_pull_up(M4_HOME_SW);
    gpio_init(M5_HOME_SW);
    gpio_pull_up(M5_HOME_SW);

    gpio_init(STEP_PIN);
    gpio_init(DIR_PIN);
    gpio_set_dir(STEP_PIN, GPIO_OUT);
    gpio_set_dir(DIR_PIN, GPIO_OUT);
}

void set_vel_mode(float mode, bool print_msg)
{
    if (mode == 1.0)
    {
        if (print_msg)
            printf("Velocity control mode on \n");
        PID_elbow.set_gains(0.0, 0.0, kd1);
        PID_wrist_left.set_gains(0.0, 0.0, kd1);
        PID_wrist_right.set_gains(0.0, 0.0, kd1);
    }
    else if (mode == 0.0)
    {
        if (print_msg)
            printf("Velocity control mode off \n");
        elbow_joint.ref_position = elbow_joint.position;
        wrist_left_joint.ref_position = wrist_left_joint.position;
        wrist_right_joint.ref_position = wrist_right_joint.position;
        PID_elbow.set_gains(kp1, ki1, kd1);
        PID_wrist_left.set_gains(kp1, ki1, kd1);
        PID_wrist_right.set_gains(kp1, ki1, kd1);
    }
    else
    {
        if (print_msg)
            printf("Wrong command! Set 1 to vel mode, 0 to pos mode \n");
    }
}

bool home_elbow()
{
    bool elbow_home = false;
    float initial_time, current_time;

    while (!elbow_home)
    {
        float theta0 = elbow_joint.position, dot_theta0 = 0, thetaf, dot_thetaf = 0, t_final;
        float a, b, c, d;

        bool switch_pressed = gpio_get(M3_HOME_SW);

        printf("Fixing elbow home \n");

        thetaf = gpio_get(M3_HOME_SW) ? 100 : -20;

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
            elbow_joint.ref_position = a + b * time_process + c * pow(time_process, 2) + d * pow(time_process, 3);
            elbow_joint.ref_velocity = b + 2 * c * time_process + 3 * d * pow(time_process, 2);
            if (!gpio_get(M3_HOME_SW) & switch_pressed)
            {
                elbow_encoder.encoder_pos = elbow_joint.ref_position = elbow_joint.ref_velocity = 0;
                elbow_home = true;
                break;
            }
            current_time = millis() / 1e3;
        }
    }
    return home_elbow;
}

bool home_wrist_pitch()
{
    bool home_wrist_pitch = false;
    float initial_time, current_time;

    while (!home_wrist_pitch)
    {
        float left_theta0 = wrist_left_joint.position, left_dot_theta0 = 0, left_thetaf, left_dot_thetaf = 0, left_t_final;
        float right_theta0 = wrist_right_joint.position, right_dot_theta0 = 0, right_thetaf, right_dot_thetaf = 0, right_t_final;

        float l_a, l_b, l_c, l_d;
        float r_a, r_b, r_c, r_d;

        bool switch_pressed = gpio_get(M4_HOME_SW);

        printf("Fixing wrist pitch home \n");

        left_thetaf = gpio_get(M4_HOME_SW) ? 200 : -60;
        right_thetaf = gpio_get(M4_HOME_SW) ? -200 : 60;

        left_t_final = abs(left_thetaf - left_theta0) / 25.0;
        l_a = left_theta0;
        l_b = left_dot_theta0;
        l_c = 3.0 * (left_thetaf - left_theta0) / pow(left_t_final, 2) - 2 * left_dot_theta0 / pow(left_t_final, 2) - left_dot_thetaf / pow(left_t_final, 2);
        l_d = -2.0 * (left_thetaf - left_theta0) / pow(left_t_final, 3) + (left_dot_thetaf - left_dot_theta0) / pow(left_t_final, 2);

        right_t_final = abs(right_thetaf - right_theta0) / 25.0;
        r_a = right_theta0;
        r_b = right_dot_theta0;
        r_c = 3.0 * (right_thetaf - right_theta0) / pow(right_t_final, 2) - 2 * right_dot_theta0 / pow(right_t_final, 2) - right_dot_thetaf / pow(right_t_final, 2);
        r_d = -2.0 * (right_thetaf - right_theta0) / pow(right_t_final, 3) + (right_dot_thetaf - right_dot_theta0) / pow(right_t_final, 2);

        initial_time = millis() / 1e3;
        current_time = initial_time;

        while ((current_time - initial_time) < right_t_final || left_t_final)
        {
            float time_process = current_time - initial_time;

            wrist_left_joint.ref_position = l_a + l_b * time_process + l_c * pow(time_process, 2) + l_d * pow(time_process, 3);
            wrist_left_joint.ref_velocity = l_b + 2 * l_c * time_process + 3 * l_d * pow(time_process, 2);

            wrist_right_joint.ref_position = r_a + r_b * time_process + r_c * pow(time_process, 2) + r_d * pow(time_process, 3);
            wrist_right_joint.ref_velocity = r_b + 2 * r_c * time_process + 3 * r_d * pow(time_process, 2);

            if (!gpio_get(M4_HOME_SW))
            {
                wrist_left_encoder.encoder_pos = wrist_left_joint.ref_position = wrist_left_joint.ref_velocity = 0;
                wrist_right_encoder.encoder_pos = wrist_right_joint.ref_position = wrist_right_joint.ref_velocity = 0;
                home_wrist_pitch = true;
                break;
            }
            current_time = millis() / 1e3;
        }
        sleep_ms(100);
    }
    return home_wrist_pitch;
}

bool home_wrist_roll()
{
    bool home_wrist_roll = false;
    float initial_time, current_time;

    while (!home_wrist_roll)
    {
        float left_theta0 = wrist_left_joint.position, left_dot_theta0 = 0, left_thetaf, left_dot_thetaf = 0, left_t_final;
        float right_theta0 = wrist_right_joint.position, right_dot_theta0 = 0, right_thetaf, right_dot_thetaf = 0, right_t_final;

        float l_a, l_b, l_c, l_d;
        float r_a, r_b, r_c, r_d;

        printf("Fixing wrist roll home \n");

        left_thetaf = gpio_get(M5_HOME_SW) ? -200 : 30;
        right_thetaf = gpio_get(M5_HOME_SW) ? -200 : 30;

        left_t_final = abs(left_thetaf - left_theta0) / 25.0;
        l_a = left_theta0;
        l_b = left_dot_theta0;
        l_c = 3.0 * (left_thetaf - left_theta0) / pow(left_t_final, 2) - 2 * left_dot_theta0 / pow(left_t_final, 2) - left_dot_thetaf / pow(left_t_final, 2);
        l_d = -2.0 * (left_thetaf - left_theta0) / pow(left_t_final, 3) + (left_dot_thetaf - left_dot_theta0) / pow(left_t_final, 2);

        right_t_final = abs(right_thetaf - right_theta0) / 25.0;
        r_a = right_theta0;
        r_b = right_dot_theta0;
        r_c = 3.0 * (right_thetaf - right_theta0) / pow(right_t_final, 2) - 2 * right_dot_theta0 / pow(right_t_final, 2) - right_dot_thetaf / pow(right_t_final, 2);
        r_d = -2.0 * (right_thetaf - right_theta0) / pow(right_t_final, 3) + (right_dot_thetaf - right_dot_theta0) / pow(right_t_final, 2);

        initial_time = millis() / 1e3;
        current_time = initial_time;

        while ((current_time - initial_time) < right_t_final || left_t_final)
        {
            float time_process = current_time - initial_time;

            wrist_left_joint.ref_position = l_a + l_b * time_process + l_c * pow(time_process, 2) + l_d * pow(time_process, 3);
            wrist_left_joint.ref_velocity = l_b + 2 * l_c * time_process + 3 * l_d * pow(time_process, 2);

            wrist_right_joint.ref_position = r_a + r_b * time_process + r_c * pow(time_process, 2) + r_d * pow(time_process, 3);
            wrist_right_joint.ref_velocity = r_b + 2 * r_c * time_process + 3 * r_d * pow(time_process, 2);

            if (!gpio_get(M5_HOME_SW))
            {
                wrist_left_encoder.encoder_pos = wrist_left_joint.ref_position = wrist_left_joint.ref_velocity = 0;
                wrist_right_encoder.encoder_pos = wrist_right_joint.ref_position = wrist_right_joint.ref_velocity = 0;
                home_wrist_roll = true;
                break;
            }
            current_time = millis() / 1e3;
        }
        sleep_ms(100);
    }
    return home_wrist_roll;
}

void print_state_joints()
{
    printf("%.3f,%.3f,%.3f\n",
           -elbow_joint.position, -wrist_left_joint.position, -wrist_right_joint.position);
}

void print_vel_joints()
{
    printf("%.3f,%.3f,%.3f\n",
           elbow_joint.velocity, wrist_left_joint.velocity, wrist_right_joint.velocity);
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

        elbow_joint.ref_position = round(-result[0] / SHOULDER_RELATION) * SHOULDER_RELATION;
        wrist_left_joint.ref_position = round(result[1] / WRIST_RELATION) * WRIST_RELATION;
        wrist_right_joint.ref_position = round(result[2] / WRIST_RELATION) * WRIST_RELATION;
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
        elbow_joint.ref_velocity = result[0];
        wrist_left_joint.ref_velocity = result[1];
        wrist_right_joint.ref_velocity = result[2];
        break;
    case (READ_ENCODER):
        print_state_joints();
        break;
    case (READ_VEL_ENCODER):
        print_vel_joints();
        break;
    case (SET_EXTRUDER):
        token = strtok(NULL, " ");

        stepper_deg = strtof(token, &previous);
        stepper_speed = strtof(previous + 1, &current);

        printf("Degrees %.3f and speed %.3f \n", stepper_deg, stepper_speed);
        break;
    case (SET_VEL_MODE):
        token = strtok(NULL, " ");
        float mode;
        mode = strtof(token, &previous);
        set_vel_mode(mode, true);
        break;
    case (HOME):
        home_elbow();
        home_wrist_roll();
        sleep_ms(200);
        home_wrist_pitch();
        elbow_encoder.encoder_pos = round(HOME_ELBOW_ANGLE / ELBOW_RELATION);
        elbow_joint.ref_position = round(HOME_ELBOW_ANGLE / ELBOW_RELATION) * ELBOW_RELATION;
        wrist_left_encoder.encoder_pos = round(HOME_WRIST_LEFT_ANGLE / WRIST_RELATION);
        wrist_left_joint.ref_position = round(HOME_WRIST_LEFT_ANGLE / WRIST_RELATION) * WRIST_RELATION;
        wrist_right_encoder.encoder_pos = round(HOME_WRIST_RIGHT_ANGLE / WRIST_RELATION);
        wrist_right_joint.ref_position = round(HOME_WRIST_RIGHT_ANGLE / WRIST_RELATION) * WRIST_RELATION;
        break;
    case (CLEAR_JOINTS):
        printf("Encoder variables cleaned! \n");
        elbow_encoder.encoder_pos = elbow_joint.ref_position = 0;
        wrist_left_encoder.encoder_pos = wrist_left_joint.ref_position = 0;
        wrist_right_encoder.encoder_pos = wrist_right_joint.ref_position = 0;
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
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
}

void updatePid(int32_t joint1_encoder_ticks, int32_t joint2_encoder_ticks, int32_t joint3_encoder_ticks)
{
    float position_elbow = float(joint1_encoder_ticks) * ELBOW_RELATION;
    float position_wrist_left = float(joint2_encoder_ticks) * WRIST_RELATION;
    float position_wrist_right = float(joint3_encoder_ticks) * WRIST_RELATION;

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

    elbow_motor.write(M3_ENC_INVERTED ? -elbow_joint.effort : elbow_joint.effort);
    wrist_left_motor.write(M4_ENC_INVERTED ? -wrist_left_joint.effort : wrist_left_joint.effort);
    wrist_right_motor.write(M5_ENC_INVERTED ? -wrist_right_joint.effort : wrist_right_joint.effort);
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
    printf("Slave control");

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    initRobot();

    multicore_launch_core1(core1_comm);
    sleep_ms(500);

    gpio_set_irq_enabled_with_callback(M3_ENC_A_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoders_callback);
    gpio_set_irq_enabled_with_callback(M4_ENC_A_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoders_callback);
    gpio_set_irq_enabled_with_callback(M5_ENC_A_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoders_callback);

    repeating_timer_t timer;
    if (!add_repeating_timer_ms(-sample_time_ms, timerCallback, NULL, &timer))
    {
        printf("Failure by not set timer!! \n");
    }

    while (true)
    {
        moveDegrees(stepper_deg, stepper_speed);
        stepper_deg = 0;
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(100);
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(100);
    }
}