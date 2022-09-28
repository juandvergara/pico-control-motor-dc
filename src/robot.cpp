//
// Created by pepe on 14/7/21.
//

#include "robot.h"

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

void Robot::updatePid(uint32_t joint1_encoder_ticks, uint32_t joint2_encoder_ticks, uint32_t joint3_encoder_ticks,)
{
    int32_t joint1_ticks = joint1_encoder_ticks;
    int32_t joint2_ticks = joint2_encoder_ticks;
    int32_t joint3_ticks = joint3_encoder_ticks;

    _state.l_position = (2.0 * M_PI) * l_ticks / ROBOT_MOTOR_PPR;
    _state.r_position = (2.0 * M_PI) * r_ticks / ROBOT_MOTOR_PPR;
    _state.r_position = (2.0 * M_PI) * r_ticks / ROBOT_MOTOR_PPR;

    int32_t djoint1_ticks = joint1_ticks - _state.joint1_ticks;
    int32_t djoint2_ticks = joint2_ticks - _state.joint2_ticks;
    int32_t djoint3_ticks = joint3_ticks - _state.joint3_ticks;

    // update odometry
    // updateOdometry(djoint1_ticks, djoint2_ticks);

    _state.joint1_ref_speed = _joint1_setpoint;
    _state.joint2_ref_speed = _joint2_setpoint;
    _state.joint3_ref_speed = _joint3_setpoint;

    /*_state.l_speed = (2.0 * M_PI) * djoint1_ticks / (ROBOT_MOTOR_PPR * _pid_rate);
    _state.r_speed = (2.0 * M_PI) * djoint2_ticks / (ROBOT_MOTOR_PPR * _pid_rate);

    _odom.v = (ROBOT_WHEEL_RADIUS / 2.0f) * (_state.l_speed + _state.r_speed);
    _odom.w = (ROBOT_WHEEL_RADIUS / ROBOT_WHEEL_SEPARATION) * (_state.r_speed - _state.l_speed);
*/
    _joint1_input = _state.joint1_speed;
    _joint2_input = _state.joint2_speed;
    _joint3_input = _state.joint3_speed;

    _joint1_pid.compute();
    _joint2_pid.compute();
    _joint3_pid.compute();

    _state.joint1_effort = _joint1_output;
    _state.joint2_effort = _joint2_output;
    _state.joint3_effort = _joint2_output;

    _joint1_motor.write(_state.joint1_effort);
    _joint2_motor.write(_state.joint2_effort);
    _joint3_motor.write(_state.joint3_effort);

    _state.joint1_ticks = joint1_ticks;
    _state.joint2_ticks = joint2_ticks;
    _state.joint3_ticks = joint3_ticks;
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



