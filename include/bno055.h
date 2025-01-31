#pragma once
#include <Arduino.h>
#include <Wire.h>
// #include <imu.h>

#include "BNO055_support.h"  //Contains the bridge code between the API and Arduino

float gxrs_bno;
float gyrs_bno;
float gzrs_bno;
float vertical_velocity = 0.1;
float roll = 0.0, pitch = 0.0, yaw = 0.0;
float gxrs = 0.0, gyrs = 0.0, gzrs = 0.0;
float yaw_sp = 0.0, last_yaw = 0.0;
float roll_error = -1.06f, pitch_error = 1.69f;
// float roll_error = 0.00, pitch_error = 0.00;
//  r:-1.19 p:2.56 y:360.00
struct bno055_t myBNO;
unsigned char accelCalibStatus = 0;  // Variable to hold the calibration status of the Accelerometer
unsigned char magCalibStatus = 0;    // Variable to hold the calibration status of the Magnetometer
unsigned char gyroCalibStatus = 0;   // Variable to hold the calibration status of the Gyroscope
unsigned char sysCalibStatus = 0;    // Variable to hold the calibration status of the System (BNO055's MCU)

unsigned long last_Time = 0;

struct bno055_euler myEulerData;
struct bno055_gyro gyroData;
struct bno055_accel accelData;

unsigned char sys_calib_status = 0;
unsigned char mag_calib_status = 0;
unsigned long last_time = 0;

void bno055_calibration() {
    if ((millis() - last_Time) >= 200)  // To read calibration status at 5 Hz without using additional timers
    {
        last_Time = millis();

        Serial.print("Time Stamp: ");  // To read out the Time Stamp
        Serial.println(last_Time);

        bno055_get_accelcalib_status(&accelCalibStatus);
        Serial.print("Accelerometer Calibration Status: ");  // To read out the Accelerometer Calibration Status (0-3)
        Serial.println(accelCalibStatus);

        bno055_get_magcalib_status(&magCalibStatus);
        Serial.print("Magnetometer Calibration Status: ");  // To read out the Magnetometer Calibration Status (0-3)
        Serial.println(magCalibStatus);

        bno055_get_magcalib_status(&gyroCalibStatus);
        Serial.print("Gyroscope Calibration Status: ");  // To read out the Gyroscope Calibration Status (0-3)
        Serial.println(gyroCalibStatus);

        bno055_get_syscalib_status(&sysCalibStatus);
        Serial.print("System Calibration Status: ");  // To read out the Magnetometer Calibration Status (0-3)
        Serial.println(sysCalibStatus);

        Serial.println();  // To separate between packets
    }
}

void check_imu_calibration() {
    while (mag_calib_status != 3 || sys_calib_status != 3) {
        if ((millis() - last_time) > 200) {
            bno055_get_magcalib_status(&mag_calib_status);
            bno055_get_syscalib_status(&sys_calib_status);
            Serial7.print("Time Stamp: ");
            Serial7.println(last_time);
            Serial7.print("Magnetometer Calibration Status: ");
            Serial7.println(mag_calib_status);
            Serial7.print("System Calibration Status: ");
            Serial7.println(sys_calib_status);
            last_time = millis();
        }
    }
}

void bno055_update() {
    // while(1){
    bno055_read_gyro_xyz(&gyroData);
    gxrs = (float(gyroData.x) / 16.0);       // in degrees/second
    gyrs = -1 * (float(gyroData.y) / 16.0);  // in degrees/second
    gzrs = (float(gyroData.z) / 16.0);       // in degrees/second
    bno055_read_euler_hrp(&myEulerData);
    roll = (float(myEulerData.r) / 16.00) - roll_error;
    pitch = (float(myEulerData.p) / 16.00) - pitch_error;
    yaw = (float(myEulerData.h) / 16.00);
    if (yaw > 180.0) {
        yaw = yaw - 360.0;
    }
    // yaw =yaw2;
    yaw_sp = yaw - last_yaw;
    last_yaw = yaw;
    bno055_read_accel_xyz(&accelData);
    // vertical_velocity = float(accelData.z) / 1000.0;
    // }
}

void bno055_init() {
    // Initialize I2C communication
    Wire2.begin();
    // Initialization of the BNO055
    BNO_Init(&myBNO);  // Assigning the structure to hold information about the device
    // Configuration to NDoF mode
    bno055_set_operation_mode(OPERATION_MODE_NDOF);
    check_imu_calibration();
    delay(1);

    // bno055_calibration();
    // threads.addThread(bno055_update);
}
