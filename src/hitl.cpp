#include <Arduino.h>
#include <communication.h>
#include <quadrotor_controller.h>

#define DT 0.01 // 10 ms
#define LOOP_INTERVAL 10000 // make sure the loop runs exactly every 10 ms

uint32_t loop_timer;
unsigned long previous_millis = 0;  // Stores the last time the setpoint was changed
unsigned long interval_on = 500;    // 2 seconds to stay at 25 degrees
unsigned long interval_off = 5000;   // 5 seconds cycle
bool is_on = false;  // Flag to check if we are in the "on" state
double disturbance;

void setup() {
    Serial.begin(9600);
    Serial7.begin(115200);
    init_gains();
    loop_timer = micros();
}

void received_data() {
    // Serial7.print("kp_roll");
    // Serial7.print(kp_roll);
    // Serial7.print(" ");
    // Serial7.print("kd_roll");
    // Serial7.print(kd_roll);
    // Serial7.print(" ");
    // Serial7.print("kp_pitch");
    // Serial7.print(kp_pitch);
    // Serial7.print(" ");
    // Serial7.print("kd_pitch");
    // Serial7.print(kd_pitch);
    // Serial7.print(" ");
    // Serial7.print("kp_yaw");
    // Serial7.print(kp_yaw);
    // Serial7.print(" ");
    // Serial7.print("kd_yaw");
    // Serial7.print(kd_yaw);
    // Serial7.print(" ");
    // Serial7.print("kp_altitude");
    // Serial7.print(kp_altitude);
    // Serial7.print(" ");
    // Serial7.print("kd_altitude");
    // Serial7.print(kd_altitude);
    // Serial7.print(" ");
    Serial7.print("roll");
    Serial7.print(roll);
    Serial7.print(" ");
    Serial7.print("roll_vel");
    Serial7.print(roll_vel);
    Serial7.print(" ");
    Serial7.print("pitch");
    Serial7.println(pitch);
    Serial7.print(" ");
    Serial7.print("pitch_setpoint");
    Serial7.print(pitch_setpoint);
    Serial7.print(" ");
    Serial7.print("roll_setpoint");
    Serial7.println(roll_setpoint);
}

void data_sent() {
    Serial7.println(torque_x);
    Serial7.println(torque_y);
    Serial7.println(torque_z);
    Serial7.println(total_force);
    
}

void loop() {  
    uint32_t current_time = micros();  // Get current time in microseconds
    float dt = (current_time - loop_timer);
    unsigned long current_millis = millis();

    if (current_millis - previous_millis >= (is_on ? interval_on : interval_off)) {
        // Save the last time a change was made
        previous_millis = current_millis;

        // Toggle the is_on flag
        is_on = !is_on;
    }

    // Set roll and pitch setpoints based on the is_on flag
    if (is_on) {
        roll_disturbance = 0.3f;  // Set to 25 degrees
        // pitch_setpoint = 25.0f; // Set to 25 degrees
    } else {
        roll_disturbance = 0.0f;   // Set to 0 degrees
        // pitch_setpoint = 0.0f;  // Set to 0 degrees
    }
    // Serial7.print("roll_disturbance");

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
            if (yaw > 180.0f){
                yaw -= 360.0f;
            }
            else if (yaw < -180.0f){
                yaw += 360.0f;
            }
            yaw_vel = msg.sensors.angular_vel_z * RAD_TO_DEG;
            altitude = msg.sensors.altitude * 100.0;
        }
        if (msg.has_command)
        {
            roll_setpoint = msg.command.roll;
            pitch_setpoint = msg.command.pitch;
        }
        alt_vel = (alt_now - alt_last) / DT;
        hitl_controller(0.0, roll_setpoint, pitch_setpoint, alt_vel, 150.0, dt);
        // received_data();
        Serial7.print(" ");
        Serial7.println(torque_x);
        msg.controls.torque_x = torque_x;
        msg.controls.torque_y = torque_y;
        msg.controls.torque_z = torque_z;
        msg.controls.total_force = total_force;
        msg.has_controls = true;
        Serial7.println(msg.controls.torque_x);
        send_message(msg);
    }

    // while (micros() - loop_timer < LOOP_INTERVAL) {}
    loop_timer = micros();
}