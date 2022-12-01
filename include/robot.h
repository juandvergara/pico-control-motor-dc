//
// Created by pepe on 14/7/21.
//

#ifndef DIFF_DRIVE_ROBOT_H
#define DIFF_DRIVE_ROBOT_H

#include "pico/stdlib.h"
#include "math.h"
#include "pid_controller.h"
#include "dc_motor.h"

#define SLIDEBASE_RELATION 0.008809710258418167f
#define BASE_RELATION 0.007047768206734534f     
#define SHOULDER_RELATION 0.008809710258418167f  
#define ELBOW_RELATION 0.008809710258418167f
#define WRIST_RELATION 0.03435114503816794f

struct MotorPins
{
    uint pwm;
    uint en_a;
    uint en_b;
};

struct RobotPins
{
    MotorPins joint1;
    MotorPins joint2;
    MotorPins joint3;
};

struct RobotState
{
    int32_t joint1_ticks;
    int32_t joint2_ticks;
    int32_t joint3_ticks;
    float joint1_position;
    float joint2_position;
    float joint3_position;
    float joint1_speed;
    float joint2_speed;
    float joint3_speed;
    float joint1_effort;
    float joint2_effort;
    float joint3_effort;
    float joint1_ref_speed;
    float joint2_ref_speed;
    float joint3_ref_speed;
};

struct RobotOdometry
{
    float x_pos;
    float y_pos;
    float theta;
    float v;
    float w;
};

class Robot{
public:
    Robot(
            float kp,
            float kd,
            float ki,
            uint32_t sample_time_ms,
            uint status_led_pin,
            RobotPins pins
            );
    void start();
    void setPosition(float joint1_position, float joint2_position, float joint3_position);
    void setUnicycle(float v, float w);
    RobotState getState();
    RobotOdometry getOdometry();
    void setPidTunings(float kp, float kd, float ki);
    void updatePid(uint32_t joint1_encoder_ticks, uint32_t joint2_encoder_ticks, uint32_t joint3_encoder_ticks);

private:
    float _kp;
    float _kd;
    float _ki;
    float _pid_rate;
    uint32_t _sample_time_ms;
    float _joint1_input;
    float _joint1_output;
    float _joint1_setpoint;
    float _joint2_input;
    float _joint2_output;
    float _joint2_setpoint;
    float _joint3_input;
    float _joint3_output;
    float _joint3_setpoint;
    float _linear;
    float _angular;

    DCMotor _joint1_motor;
    DCMotor _joint2_motor;
    DCMotor _joint3_motor;
    PID _joint1_pid;
    PID _joint2_pid;
    PID _joint3_pid;
    uint _status_led_pin;
    RobotState _state;
    RobotOdometry _odom;

    void controlLoop();
    void updateOdometry(int32_t dl_ticks, int32_t dr_ticks);
    void initPins();
};


#endif //DIFF_DRIVE_ROBOT_H
