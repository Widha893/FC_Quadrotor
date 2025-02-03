#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include <Arduino.h>
#include <math.h>
#include "Radio.h"
#include "bno055.h"
#define INTEGRAL_LIMIT 10.0f
#define MAX_VALUE 50.0f
#define MIN_VALUE -50.0f
#define MIN_THRUST 30.0f
#define MAX_MOTOR_SPEED 1025.38f
#define MIN_MOTOR_SPEED 0.0f
#define MAX_VALUE_YAW 50.0f
#define MIN_VALUE_YAW -50.0f
#define MAX_ROLL 25.0f
#define MAX_PITCH 25.0f
#define MAX_YAW 25.0f
#define MIN_PWM 1000
#define MAX_PWM 2000
#define MAX_THRUST 18.24f

float u1, u2, u3, u4;
float roll_integrator, pitch_integrator, yaw_integrator, altitude_integrator;
float setpoint_roll, setpoint_pitch, setpoint_yaw, target_alt;
float setpoint_roll_last, setpoint_pitch_last, setpoint_yaw_last, target_alt_last;
float setpoint_roll_now, setpoint_pitch_now, setpoint_yaw_now, target_alt_now;
float setpoint_roll_rate, setpoint_pitch_rate, setpoint_yaw_rate;
float state_yaw;
float altitude;
float alt_last, alt_now, alt_vel;
float ki_roll = 0.0f;
float ki_pitch = 0.0f;
float motor_speed[4];
float motor_thrust[4];
float motor_speed_rpm[4];
float motor_speed_squared[4];
float motor1_pwm, motor2_pwm, motor3_pwm, motor4_pwm;
uint8_t t_now, t_last, dt;

// Hitung Lagi!!!
// const double A_invers[4][4] = {{292600, -1300300,  1300300,  6283300},
//                                {292600, -1300300, -1300300, -6283300},
//                                {292600,  1300300, -1300300,  6283300},
//                                {292600,  1300300,  1300300, -6283300}};

// const double A_invers[4][4] = {{292600, -1300300, -1300300,  6283300},
//                                {292600,  1300300, -1300300, -6283300},
//                                {292600,  1300300,  1300300,  6283300},
//                                {292600, -1300300,  1300300, -6283300}};

const double A_invers[4][4] = {{ 1898.32310869,  -8436.9915942,  -8436.9915942,  36597.04602088 },
                               { 1898.32310869,   8436.9915942,  -8436.9915942, -36597.04602088 },
                               { 1898.32310869,   8436.9915942,   8436.9915942,  36597.04602088 },
                               { 1898.32310869,  -8436.9915942,   8436.9915942, -36597.04602088 }};

// const double conversion_matrix[4][4] = {{ 0.25,   -0.25,   -0.25,    4.81695568},
//                                         { 0.25,    0.25,   -0.25,   -4.81695568},
//                                         { 0.25,    0.25,    0.25,    4.81695568},
//                                         { 0.25,   -0.25,    0.25,   -4.81695568}};

struct Gains {
    float alt = 0.0f;
    float vz = 0.0f;
    float roll = 5.916;
    float p = 4.144;
    float pitch = 5.196;
    float q = 3.341;
    float yaw = 0.0; // 6.324
    float r = 0.0; // 1.160
} gain;

float constrain_value(float value, float min, float max) {
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

void drone_controller() {
    t_now = micros();
    dt = t_now - t_last;
    t_last = t_now;

    setpoint_roll_last = setpoint_roll_now;
    setpoint_roll_now = setpoint_roll;

    setpoint_pitch_last = setpoint_pitch_now;
    setpoint_pitch_now = setpoint_pitch;

    setpoint_yaw_last = setpoint_yaw_now;
    setpoint_yaw_now = setpoint_yaw;

    if (dt > 0) {
        setpoint_roll_rate = (setpoint_roll_now - setpoint_roll_last) / dt;
        setpoint_pitch_rate = (setpoint_pitch_now - setpoint_pitch_last) / dt;
        setpoint_yaw_rate = (setpoint_yaw_now - setpoint_yaw_last) / dt;
    } else {
        setpoint_roll_rate = 0.0f;
        setpoint_pitch_rate = 0.0f;
        setpoint_yaw_rate = 0.0f;
    }

    alt_last = alt_now;
    alt_now = altitude;
    alt_vel = (alt_now - alt_last) / dt;

    float error_roll = roll - setpoint_roll;
    float error_roll_rate = gxrs - setpoint_roll_rate;
    float error_pitch = pitch - setpoint_pitch;
    float error_pitch_rate = gyrs - setpoint_pitch_rate;
    float error_yaw = yaw - setpoint_yaw;
    float error_yaw_rate = gzrs - setpoint_yaw_rate;
    float error_altitude = target_alt - altitude;

    /* u = -k * (state - setpoint) */
    float p_roll = -gain.roll * error_roll;
    float d_roll = -gain.p * error_roll_rate;
    float p_pitch = -gain.pitch * error_pitch;
    float d_pitch = -gain.q * error_pitch_rate;
    float p_yaw = -gain.yaw * error_yaw;
    float d_yaw = -gain.r * error_yaw_rate;

    u1 = 0.0; // maximum thrust of x2216 skywalker x 4 using 1047 prop and 4s battery in newton //0.0;//(gain.alt * error_altitude) + (gain.vz * alt_vel);
    u2 = (p_roll + d_roll)/10000.0f;
    u3 = (p_pitch + d_pitch)/10000.0f;
    u4 = (p_yaw + d_yaw)/10000.0f;

    // u1 = max(u1, MIN_THRUST);
    // u2 = constrain_value(u2, MIN_VALUE, MAX_VALUE);
    // u3 = constrain_value(u3, MIN_VALUE, MAX_VALUE);
    // u4 = constrain_value(u4, MIN_VALUE_YAW, MAX_VALUE_YAW);

    motor_speed_squared[0] = ((A_invers[0][0] * u1 + A_invers[0][1] * u2 + A_invers[0][2] * u3 + A_invers[0][3] * u4));
    motor_speed_squared[1] = ((A_invers[1][0] * u1 + A_invers[1][1] * u2 + A_invers[1][2] * u3 + A_invers[1][3] * u4));
    motor_speed_squared[2] = ((A_invers[2][0] * u1 + A_invers[2][1] * u2 + A_invers[2][2] * u3 + A_invers[2][3] * u4));
    motor_speed_squared[3] = ((A_invers[3][0] * u1 + A_invers[3][1] * u2 + A_invers[3][2] * u3 + A_invers[3][3] * u4));

    // motor_speed_squared[0] = abs(motor_speed_squared[0]);
    // motor_speed_squared[1] = abs(motor_speed_squared[1]);
    // motor_speed_squared[2] = abs(motor_speed_squared[2]);
    // motor_speed_squared[3] = abs(motor_speed_squared[3]);

    // motor_speed[0] = sqrt(motor_speed_squared[0]);
    // motor_speed[1] = sqrt(motor_speed_squared[1]);
    // motor_speed[2] = sqrt(motor_speed_squared[2]);
    // motor_speed[3] = sqrt(motor_speed_squared[3]);

    // motor_speed[0] = constrain_value(motor_speed[0], MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
    // motor_speed[1] = constrain_value(motor_speed[1], MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
    // motor_speed[2] = constrain_value(motor_speed[2], MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
    // motor_speed[3] = constrain_value(motor_speed[3], MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);

    // motor_thrust[0] = ((conversion_matrix[0][0] * u1) + (conversion_matrix[0][1] * u2) + (conversion_matrix[0][2] * u3) + (conversion_matrix[0][3] * u4));
    // motor_thrust[1] = ((conversion_matrix[1][0] * u1) + (conversion_matrix[1][1] * u2) + (conversion_matrix[1][2] * u3) + (conversion_matrix[1][3] * u4));
    // motor_thrust[2] = ((conversion_matrix[2][0] * u1) + (conversion_matrix[2][1] * u2) + (conversion_matrix[2][2] * u3) + (conversion_matrix[2][3] * u4));
    // motor_thrust[3] = ((conversion_matrix[3][0] * u1) + (conversion_matrix[3][1] * u2) + (conversion_matrix[3][2] * u3) + (conversion_matrix[3][3] * u4));

    // Conversion from motor speed to PWM - Calculated by viewing motor characteristics based on datasheet
    // 1000 + (((rad/s) / 1025.38) * 1000)
    // motor1_pwm = ch_throttle + ((motor_speed[0] / MAX_MOTOR_SPEED) * (2000 - ch_throttle));
    // motor2_pwm = ch_throttle + ((motor_speed[1] / MAX_MOTOR_SPEED) * (2000 - ch_throttle));
    // motor3_pwm = ch_throttle + ((motor_speed[2] / MAX_MOTOR_SPEED) * (2000 - ch_throttle));
    // motor4_pwm = ch_throttle + ((motor_speed[3] / MAX_MOTOR_SPEED) * (2000 - ch_throttle));

    motor1_pwm = ch_throttle + (int)(motor_speed_squared[0]);
    motor2_pwm = ch_throttle + (int)(motor_speed_squared[1]);
    motor3_pwm = ch_throttle + (int)(motor_speed_squared[2]);
    motor4_pwm = ch_throttle + (int)(motor_speed_squared[3]);

    motor1_pwm = constrain_value(motor1_pwm, MIN_PWM, MAX_PWM);
    motor2_pwm = constrain_value(motor2_pwm, MIN_PWM, MAX_PWM);
    motor3_pwm = constrain_value(motor3_pwm, MIN_PWM, MAX_PWM);
    motor4_pwm = constrain_value(motor4_pwm, MIN_PWM, MAX_PWM);

}

void set_control_reference() {
    setpoint_roll = roll_scaler() * MAX_ROLL;
    setpoint_pitch = pitch_scaler() * MAX_PITCH;
    setpoint_yaw = yaw_scaler() * MAX_YAW;
    // setpoint_yaw = yaw_sp;
//     if (setpoint_yaw > 180.0f) { setpoint_yaw -= 360.0f; }
//     if (setpoint_yaw < -180.0f) { setpoint_yaw += 360.0f; }
}

#endif