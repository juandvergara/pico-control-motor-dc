#include "pico/stdlib.h"
#include <cmath>
#include <stdio.h>
#include "dc_motor.h"
#include "pid_controller.h"
#include "encoder_cuadratura.h"

DCMotor motor1(M1_ENA_PIN, M1_ENB_PIN, M1_PWM_PIN);
DCMotor motor2(M2_ENA_PIN, M2_ENB_PIN, M2_PWM_PIN);
DCMotor motor3(M3_ENA_PIN, M3_ENB_PIN, M3_PWM_PIN);

float kp = 60.0;
float kd = 5.0;
float ki = 15.0;

float joint_input, joint_effort, joint_setpoint = 0.0;
float joint_position;
uint32_t sample_time_ms = 20;
float pid_rate;
char user_setpoint;

char in_buffer[100];
uint16_t char_idx = 0;

PID PID_Joint1(&joint_input, &joint_effort, &joint_setpoint, kp, ki, kd, sample_time_ms);

uint32_t millis()
{
    return to_ms_since_boot(get_absolute_time());
}

void initRobot()
{
    motor1.write(0.0);
    // PID PID_Joint1(&joint_input, &joint_effort, &joint_setpoint, kp, ki, kd, sample_time_ms);
    PID_Joint1.set_output_limits(-90.0f, 90.0f);
    joint_setpoint = 0;
    pid_rate = float(sample_time_ms) / 1000.0f;
}

void updatePid(int32_t joint_encoder_ticks)
{
    int32_t joint_ticks = joint_encoder_ticks;
    float motor_vel = 0;

    joint_position = float(joint_ticks) * 360.0f / (80.0f * 127.7f * 4.0f);

    // joint_setpoint = float(user_setpoint);

    joint_input = joint_position;

    PID_Joint1.compute();

    if (joint_effort < 0)
    {
        motor_vel = (-(abs(joint_effort) + 90) / 180) * 0.60 - 0.4;
    }
    else
    {
        motor_vel = ((joint_effort + 90) / 180) * 0.6 + 0.4;
    }

    motor1.write(-motor_vel);
}

int main()
{
    stdio_init_all();
    printf("Encoder Test");
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    initRobot();

    encoder_setup();
    char user_setpoin;

    int input_char;
    int input_char_index;
    char *char_pt1;
    char *char_pt2;
    char *char_pt3;

    float angular = 0.0;
    float linear = 0.0;

    while (true)
    {
        // printf("Introduce el setpoint en formato linear,angular/\n");

        input_char = getchar_timeout_us(0); // Esperar la entrada del usuario
        // printf("%c \n", input_char);
        while (input_char != PICO_ERROR_TIMEOUT)
        {
            // gpio_put(LED_PIN, true);
            // printf(" %c ", ch);
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            putchar(input_char);                        // Print user input in console
            in_buffer[input_char_index++] = input_char; // Index user input to buffer array
            if (input_char == '/')
            {
                in_buffer[input_char_index] = 0; // end of string
                printf("\nreceived: %s\n", in_buffer);
                input_char_index = 0;
                linear = strtof(in_buffer, &char_pt1);     // Conversion string (char) to float
                angular = strtof(char_pt1 + 1, &char_pt2); // Add 1 to bring up the comma
                printf("Primer parámetro %.1f y el segundo parámetro es %.1f \n", linear, angular);
                break;
            }

            input_char = getchar_timeout_us(0);
            printf("\n Caracter recibido \n");
            joint_setpoint = angular; //
            // printf("%.1f", joint_setpoint);
        }
        // gpio_put(LED_PIN, false);
        // printf("Entradas recibidas");

        // sleep_ms(100);
        //  Pedir por serial a posicion que se quiere
        updatePid(int32_t(PosEncoderM1));
        printf("El esfuerzo es de %.3f\n", joint_effort / 1.0);
        printf("La posición es de %.3f\n", joint_position);
        printf("El setpoint es de %.3f\n \n", joint_setpoint);
        // sleep_ms(100);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
    }
}