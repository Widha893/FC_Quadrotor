#ifndef QUADROTOR_CONTROLLER_H
#define QUADROTOR_CONTROLLER_H

#include <Arduino.h>
#include <communication.h>
#define INTEGRAL_LIMIT 10.0f

double torque_x, torque_y, torque_z, total_force;
double roll_integrator, pitch_integrator, yaw_integrator, altitude_integrator;
double kp_roll, kd_roll, kp_pitch, kd_pitch, kp_yaw, kd_yaw, kp_altitude, kd_altitude;
double roll, roll_vel, pitch, pitch_vel, yaw, yaw_vel, altitude;
double alt_last, alt_now, alt_vel;
double ki_roll = 0.0;
double ki_pitch = 0.0;

void init_gains() {
    kp_roll = 0.0;
    kd_roll = 0.0;
    kp_pitch = 0.0;
    kd_pitch = 0.0;
    kp_yaw = 0.0;
    kd_yaw = 0.0;
    kp_altitude = 0.0;
    kd_altitude = 0.0;
}

void hitl_controller(double setpoint, double vz, double target_alt, uint8_t dt) {
    alt_last = alt_now;
    alt_now = altitude;
    // alt_vel = (alt_now - alt_last) / dt;
    double error_roll = setpoint - roll;
    double error_pitch = setpoint - pitch;
    double error_yaw = setpoint - yaw;
    double error_altitude = target_alt - altitude;

    if (abs(error_roll) < 10.0f) {
        roll_integrator += error_roll * dt;
        roll_integrator = constrain(roll_integrator, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    }
    if (abs(error_pitch) < 10.0f) {
        pitch_integrator += error_pitch * dt;
        pitch_integrator = constrain(pitch_integrator, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    }

    total_force = (kp_altitude * error_altitude) + (kd_altitude * alt_vel);
    torque_x = (kp_roll * error_roll) + (kd_roll * roll_vel) + (roll_integrator*ki_roll);
    torque_y = (kp_pitch * error_pitch) + (kd_pitch * pitch_vel) + (pitch_integrator*ki_pitch);
    torque_z = (kp_yaw * error_yaw) + (kd_yaw * yaw_vel);
}

#endif