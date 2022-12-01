//
// Created by pepe on 14/7/21.
//

#include "robot.h"
#include "pins.h"
#include "dc_motor_v2.h"
#include "encoder.h"

Robot::Robot(
        float kp,
        float kd,
        float ki,
        uint32_t sample_time_ms,
        uint status_led_pin,
        RobotPins pins
        )
:
_kp(kp), _kd(kd), _ki(ki), _sample_time_ms(sample_time_ms),
_joint1_input(0.0f), _joint1_output(0.0f), _joint1_setpoint(0.0f),
_joint2_input(0.0f), _joint2_output(0.0f), _joint2_setpoint(0.0f),
_joint3_input(0.0f), _joint3_output(0.0f), _joint3_setpoint(0.0f),
_joint1_motor(pins.joint1.en_a, pins.joint1.en_b, pins.joint1.pwm),
_joint2_motor(pins.joint2.en_a, pins.joint2.en_b, pins.joint2.pwm),
_joint3_motor(pins.joint3.en_a, pins.joint3.en_b, pins.joint3.pwm),
_joint1_pid(&_joint1_input, &_joint2_output, &_joint2_setpoint, kp, ki, kd, sample_time_ms),
_joint2_pid(&_joint2_input, &_joint2_output, &_joint2_setpoint, kp, ki, kd, sample_time_ms),
_joint3_pid(&_joint3_input, &_joint3_output, &_joint3_setpoint, kp, ki, kd, sample_time_ms),
_status_led_pin(status_led_pin)
{
    _joint1_motor.write(0.0f);
    _joint2_motor.write(0.0f);
    _joint3_motor.write(0.0f);
    _joint1_pid.set_output_limits(-1.0f, 1.0f);
    _joint2_pid.set_output_limits(-1.0f, 1.0f);
    _joint3_pid.set_output_limits(-1.0f, 1.0f);
    _joint1_setpoint = 0;
    _joint2_setpoint = 0;
    _joint3_setpoint = 0;
    _pid_rate = float(sample_time_ms) / 1000.0f;
    initPins();
}
void Robot::updatePid(int32_t joint1_encoder_ticks, int32_t joint2_encoder_ticks, int32_t joint3_encoder_ticks)
{
    int32_t joint1_ticks = joint1_encoder_ticks;
    int32_t joint2_ticks = joint2_encoder_ticks;
    int32_t joint3_ticks = joint3_encoder_ticks;

    float motor1_vel = 0;
    float motor2_vel = 0;
    float motor3_vel = 0;

    float position_elbow = float(joint1_ticks) * 360.0f / (80.0f * 127.7f * 4.0f);
    float position_wrist_left = float(joint2_ticks) * 360.0f * 23.0f / (80.0f * 65.5f * 32.0f);
    float position_wrist_right = float(joint3_ticks) * 360.0f * 23.0f / (80.0f * 65.5f * 32.0f);

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

    PID_Joint1.compute();
    PID_Joint2.compute();
    PID_Joint3.compute();

    motor1_vel = elbow_joint.effort;
    motor2_vel = wrist_left_joint.effort;
    motor3_vel = wrist_right_joint.effort;

    M3_ENC_INVERTED ? motor1_vel = -elbow_joint.effort : motor1_vel = elbow_joint.effort;
    M4_ENC_INVERTED ? motor2_vel = -wrist_left_joint.effort : motor2_vel = wrist_left_joint.effort;
    M5_ENC_INVERTED ? motor3_vel = -wrist_right_joint.effort : motor3_vel = wrist_right_joint.effort;

    motor3.write(-motor1_vel);
    motor4.write(motor2_vel);
    motor5.write(-motor3_vel);
}

void Robot::updateOdometry(int32_t dl_ticks, int32_t dr_ticks) {
    float delta_l = (2 * M_PI * ROBOT_WHEEL_RADIUS * dl_ticks) / ROBOT_MOTOR_PPR;
    float delta_r = (2 * M_PI * ROBOT_WHEEL_RADIUS * dr_ticks) / ROBOT_MOTOR_PPR;
    float delta_center = (delta_l + delta_r) / 2;

    _odom.x_pos += delta_center * cosf(_odom.theta);
    _odom.y_pos += delta_center * sinf(_odom.theta);
    _odom.theta += (delta_r - delta_l) / ROBOT_WHEEL_SEPARATION;
    _odom.v = _linear;
    _odom.w = _angular;
}

void Robot::setPosition(float joint1_position, float joint2_position, float joint3_position)
{
    _joint1_setpoint = joint1_position;
    _joint2_setpoint = joint2_position;
    _joint3_setpoint = joint3_position;
}

void Robot::setUnicycle(float v, float w)
{
    // limit values
    if(v > ROBOT_MAX_LINEAR_M_S) v = ROBOT_MAX_LINEAR_M_S;
    if(v < ROBOT_MIN_LINEAR_M_S) v = ROBOT_MIN_LINEAR_M_S;
    if(w > ROBOT_MAX_ANGULAR_R_S) w = ROBOT_MAX_ANGULAR_R_S;
    if(w < ROBOT_MIN_ANGULAR_R_S) w = ROBOT_MIN_ANGULAR_R_S;

    float v_l = (2 * v - w * ROBOT_WHEEL_SEPARATION) / (2 * ROBOT_WHEEL_RADIUS);
    float v_r = (2 * v + w * ROBOT_WHEEL_SEPARATION) / (2 * ROBOT_WHEEL_RADIUS);

    _linear = v;
    _angular = w;
    setWheels(v_l, v_r);
}

void Robot::initPins()
{
    gpio_init(_status_led_pin);
    gpio_set_dir(_status_led_pin, GPIO_OUT);
}

RobotState Robot::getState() {
    return _state;
}

RobotOdometry Robot::getOdometry() {
    return _odom;
}