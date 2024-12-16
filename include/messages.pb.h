/* Automatically generated nanopb header */
/* Generated by nanopb-1.0.0-dev */

#ifndef PB_HWIL_MESSAGES_PB_H_INCLUDED
#define PB_HWIL_MESSAGES_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Struct definitions */
typedef struct _HWIL_msg_Gains {
    double alt;
    double vz;
    double roll;
    double p;
    double pitch;
    double q;
    double yaw;
    double r;
} HWIL_msg_Gains;

typedef struct _HWIL_msg_SensorData {
    double roll;
    double pitch;
    double yaw;
    double angular_vel_x;
    double angular_vel_y;
    double angular_vel_z;
    double altitude;
} HWIL_msg_SensorData;

typedef struct _HWIL_msg_ControlData {
    double torque_x;
    double torque_y;
    double torque_z;
    double total_force;
} HWIL_msg_ControlData;

typedef struct _HWIL_msg_Command {
    double roll;
    double pitch;
    double yaw;
} HWIL_msg_Command;

typedef struct _HWIL_msg {
    bool has_gains;
    HWIL_msg_Gains gains;
    bool has_controls;
    HWIL_msg_ControlData controls;
    bool has_sensors;
    HWIL_msg_SensorData sensors;
    bool has_command;
    HWIL_msg_Command command;
} HWIL_msg;


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define HWIL_msg_init_default                    {false, HWIL_msg_Gains_init_default, false, HWIL_msg_ControlData_init_default, false, HWIL_msg_SensorData_init_default, false, HWIL_msg_Command_init_default}
#define HWIL_msg_Gains_init_default              {0, 0, 0, 0, 0, 0, 0, 0}
#define HWIL_msg_SensorData_init_default         {0, 0, 0, 0, 0, 0, 0}
#define HWIL_msg_ControlData_init_default        {0, 0, 0, 0}
#define HWIL_msg_Command_init_default            {0, 0, 0}
#define HWIL_msg_init_zero                       {false, HWIL_msg_Gains_init_zero, false, HWIL_msg_ControlData_init_zero, false, HWIL_msg_SensorData_init_zero, false, HWIL_msg_Command_init_zero}
#define HWIL_msg_Gains_init_zero                 {0, 0, 0, 0, 0, 0, 0, 0}
#define HWIL_msg_SensorData_init_zero            {0, 0, 0, 0, 0, 0, 0}
#define HWIL_msg_ControlData_init_zero           {0, 0, 0, 0}
#define HWIL_msg_Command_init_zero               {0, 0, 0}

/* Field tags (for use in manual encoding/decoding) */
#define HWIL_msg_Gains_alt_tag                   1
#define HWIL_msg_Gains_vz_tag                    2
#define HWIL_msg_Gains_roll_tag                  3
#define HWIL_msg_Gains_p_tag                     4
#define HWIL_msg_Gains_pitch_tag                 5
#define HWIL_msg_Gains_q_tag                     6
#define HWIL_msg_Gains_yaw_tag                   7
#define HWIL_msg_Gains_r_tag                     8
#define HWIL_msg_SensorData_roll_tag             1
#define HWIL_msg_SensorData_pitch_tag            2
#define HWIL_msg_SensorData_yaw_tag              3
#define HWIL_msg_SensorData_angular_vel_x_tag    4
#define HWIL_msg_SensorData_angular_vel_y_tag    5
#define HWIL_msg_SensorData_angular_vel_z_tag    6
#define HWIL_msg_SensorData_altitude_tag         7
#define HWIL_msg_ControlData_torque_x_tag        1
#define HWIL_msg_ControlData_torque_y_tag        2
#define HWIL_msg_ControlData_torque_z_tag        3
#define HWIL_msg_ControlData_total_force_tag     4
#define HWIL_msg_Command_roll_tag                1
#define HWIL_msg_Command_pitch_tag               2
#define HWIL_msg_Command_yaw_tag                 3
#define HWIL_msg_gains_tag                       1
#define HWIL_msg_controls_tag                    2
#define HWIL_msg_sensors_tag                     3
#define HWIL_msg_command_tag                     4

/* Struct field encoding specification for nanopb */
#define HWIL_msg_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  gains,             1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  controls,          2) \
X(a, STATIC,   OPTIONAL, MESSAGE,  sensors,           3) \
X(a, STATIC,   OPTIONAL, MESSAGE,  command,           4)
#define HWIL_msg_CALLBACK NULL
#define HWIL_msg_DEFAULT NULL
#define HWIL_msg_gains_MSGTYPE HWIL_msg_Gains
#define HWIL_msg_controls_MSGTYPE HWIL_msg_ControlData
#define HWIL_msg_sensors_MSGTYPE HWIL_msg_SensorData
#define HWIL_msg_command_MSGTYPE HWIL_msg_Command

#define HWIL_msg_Gains_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, DOUBLE,   alt,               1) \
X(a, STATIC,   REQUIRED, DOUBLE,   vz,                2) \
X(a, STATIC,   REQUIRED, DOUBLE,   roll,              3) \
X(a, STATIC,   REQUIRED, DOUBLE,   p,                 4) \
X(a, STATIC,   REQUIRED, DOUBLE,   pitch,             5) \
X(a, STATIC,   REQUIRED, DOUBLE,   q,                 6) \
X(a, STATIC,   REQUIRED, DOUBLE,   yaw,               7) \
X(a, STATIC,   REQUIRED, DOUBLE,   r,                 8)
#define HWIL_msg_Gains_CALLBACK NULL
#define HWIL_msg_Gains_DEFAULT NULL

#define HWIL_msg_SensorData_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, DOUBLE,   roll,              1) \
X(a, STATIC,   REQUIRED, DOUBLE,   pitch,             2) \
X(a, STATIC,   REQUIRED, DOUBLE,   yaw,               3) \
X(a, STATIC,   REQUIRED, DOUBLE,   angular_vel_x,     4) \
X(a, STATIC,   REQUIRED, DOUBLE,   angular_vel_y,     5) \
X(a, STATIC,   REQUIRED, DOUBLE,   angular_vel_z,     6) \
X(a, STATIC,   REQUIRED, DOUBLE,   altitude,          7)
#define HWIL_msg_SensorData_CALLBACK NULL
#define HWIL_msg_SensorData_DEFAULT NULL

#define HWIL_msg_ControlData_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, DOUBLE,   torque_x,          1) \
X(a, STATIC,   REQUIRED, DOUBLE,   torque_y,          2) \
X(a, STATIC,   REQUIRED, DOUBLE,   torque_z,          3) \
X(a, STATIC,   REQUIRED, DOUBLE,   total_force,       4)
#define HWIL_msg_ControlData_CALLBACK NULL
#define HWIL_msg_ControlData_DEFAULT NULL

#define HWIL_msg_Command_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, DOUBLE,   roll,              1) \
X(a, STATIC,   REQUIRED, DOUBLE,   pitch,             2) \
X(a, STATIC,   REQUIRED, DOUBLE,   yaw,               3)
#define HWIL_msg_Command_CALLBACK NULL
#define HWIL_msg_Command_DEFAULT NULL

extern const pb_msgdesc_t HWIL_msg_msg;
extern const pb_msgdesc_t HWIL_msg_Gains_msg;
extern const pb_msgdesc_t HWIL_msg_SensorData_msg;
extern const pb_msgdesc_t HWIL_msg_ControlData_msg;
extern const pb_msgdesc_t HWIL_msg_Command_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define HWIL_msg_fields &HWIL_msg_msg
#define HWIL_msg_Gains_fields &HWIL_msg_Gains_msg
#define HWIL_msg_SensorData_fields &HWIL_msg_SensorData_msg
#define HWIL_msg_ControlData_fields &HWIL_msg_ControlData_msg
#define HWIL_msg_Command_fields &HWIL_msg_Command_msg

/* Maximum encoded size of messages (where known) */
#define HWIL_MESSAGES_PB_H_MAX_SIZE              HWIL_msg_size
#define HWIL_msg_Command_size                    27
#define HWIL_msg_ControlData_size                36
#define HWIL_msg_Gains_size                      72
#define HWIL_msg_SensorData_size                 63
#define HWIL_msg_size                            206

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
