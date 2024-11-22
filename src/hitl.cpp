#include <Arduino.h>
#include <communication.h>
#include <quadrotor_controller.h>

#define DT 0.01 // 10 ms
#define LOOP_INTERVAL 10000 // make sure the loop runs exactly every 10 ms

uint32_t loop_timer;

void setup() {
    Serial.begin(9600);
    Serial7.begin(115200);
    init_gains();
    loop_timer = micros();
}

void loop() {  
    if (receive_message()) {
        if (msg.has_gains) {
            kp_roll = msg.gains.roll;
            kd_roll = msg.gains.p;
            kp_pitch = msg.gains.pitch;
            kd_pitch = msg.gains.q;
            kp_yaw = msg.gains.yaw;
            kd_yaw = msg.gains.r;
            kp_altitude = msg.gains.alt;
            kd_altitude = msg.gains.vz;
        }
        if (msg.has_sensors)
        {
            roll = msg.sensors.roll;
            roll_vel = msg.sensors.angular_vel_x * RAD_TO_DEG;
            pitch = msg.sensors.pitch;
            pitch_vel = msg.sensors.angular_vel_y * RAD_TO_DEG;
            yaw = msg.sensors.yaw;
            yaw_vel = msg.sensors.angular_vel_z * RAD_TO_DEG;
            altitude = msg.sensors.altitude * 100.0;
        }
        alt_vel = (alt_now - alt_last) / DT;
        hitl_controller(0.0, alt_vel, 150.0, DT);
        msg.controls.torque_x = torque_x;
        msg.controls.torque_y = torque_y;
        msg.controls.torque_z = torque_z;
        msg.controls.total_force = total_force;
        Serial7.println(torque_x);
        Serial7.println(torque_y);
        Serial7.println(torque_z);
        Serial7.println(alt_now);
        Serial7.println(alt_last);
        Serial7.println(alt_vel);
        Serial7.println(total_force);
    }
    while (micros() - loop_timer < LOOP_INTERVAL) {}
    loop_timer = micros();
}