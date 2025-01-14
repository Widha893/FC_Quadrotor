#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include <Arduino.h>
#include <math.h>
#include "Radio.h"
#include "bno055.h"
#define INTEGRAL_LIMIT 10.0f
#define MAX_VALUE 30.0f
#define MIN_VALUE -30.0f
#define MAX_VALUE_YAW 10.0f
#define MIN_VALUE_YAW -10.0f
#define MAX_ROLL 30.0f
#define MAX_PITCH 30.0f
#define MAX_YAW 30.0f

float u1, u2, u3, u4;
float roll_integrator, pitch_integrator, yaw_integrator, altitude_integrator;
float setpoint_roll, setpoint_pitch, setpoint_yaw, target_alt;
float altitude;
float roll_setpoint, pitch_setpoint, roll_disturbance, pitch_disturbance;
float alt_last, alt_now, alt_vel;
float ki_roll = 0.0f;
float ki_pitch = 0.0f;
float motor_speed[4];
float motor1_pwm, motor2_pwm, motor3_pwm, motor4_pwm;

// Hitung Lagi!!!
const double A_invers[4][4] = {{292600, -1300300,  1300300,  6283300},
                               {292600, -1300300, -1300300, -6283300},
                               {292600,  1300300, -1300300,  6283300},
                               {292600,  1300300,  1300300, -6283300}};

struct Gains {
    float alt = 0.0f;
    float vz = 0.0f;
    float roll = 6.324;
    float p = 4.296;
    float pitch = 5.477;
    float q = 2.874;
    float yaw = 6.856;// 6.856
    float r = 1.729f;//1.729
} gain;

void hitl_controller() {
    alt_last = alt_now;
    alt_now = altitude;
    // alt_vel = (alt_now - alt_last) / dt;
    float error_roll = setpoint_roll - roll;
    float error_pitch = setpoint_pitch - pitch;
    float error_yaw = setpoint_yaw - yaw;
    float error_altitude = target_alt - altitude;

    // if (abs(error_roll) < 10.0f) {
    //     roll_integrator += error_roll * dt;
    //     roll_integrator = constrain(roll_integrator, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    // }
    // if (abs(error_pitch) < 10.0f) {
    //     pitch_integrator += error_pitch * dt;
    //     pitch_integrator = constrain(pitch_integrator, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    // }
    float p_roll = gain.roll * error_roll;
    float d_roll = gain.p * (gxrs);
    float p_pitch = gain.pitch * error_pitch;
    float d_pitch = gain.q * (gyrs);
    float p_yaw = gain.yaw * error_yaw;
    float d_yaw = gain.r * (gzrs);

    u1 = (gain.alt * error_altitude) + (gain.vz * alt_vel);
    u2 = p_roll + d_roll;
    u3 = p_pitch + d_pitch;
    u4 = p_yaw;

    u2 = constrain_value(u2, MIN_VALUE, MAX_VALUE);
    u3 = constrain_value(u3, MIN_VALUE, MAX_VALUE);
    u4 = constrain_value(u4, MIN_VALUE_YAW, MAX_VALUE_YAW);

    motor_speed[0] = (A_invers[0][0] * u1 + A_invers[0][1] * u2 + A_invers[0][2] * u3 + A_invers[0][3] * u4);
    motor_speed[1] = (A_invers[1][0] * u1 + A_invers[1][1] * u2 + A_invers[1][2] * u3 + A_invers[1][3] * u4);
    motor_speed[2] = (A_invers[2][0] * u1 + A_invers[2][1] * u2 + A_invers[2][2] * u3 + A_invers[2][3] * u4);
    motor_speed[3] = (A_invers[3][0] * u1 + A_invers[3][1] * u2 + A_invers[3][2] * u3 + A_invers[3][3] * u4);

    motor1_pwm = 1000 + (60.0f * motor_speed[0] / 61.5124f);
    motor2_pwm = 1000 + (60.0f * motor_speed[1] / 61.5124f);
    motor3_pwm = 1000 + (60.0f * motor_speed[2] / 61.5124f);
    motor4_pwm = 1000 + (60.0f * motor_speed[3] / 61.5124f);
}

float constrain_value(float value, float min, float max) {
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

void set_control_reference() {
    setpoint_roll = roll_scaler() * MAX_ROLL;
    setpoint_pitch = pitch_scaler() * MAX_PITCH;
    setpoint_yaw = yaw_scaler() * MAX_YAW;
}

#endif