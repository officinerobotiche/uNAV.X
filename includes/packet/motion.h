/*
 * File:   motion.h
 * Author: raffaello
 *
 * Created on 27 September 2013, 18:45
 */

#ifndef MOTION_H
#define	MOTION_H

#define BUFFER_ODOMETRY 100

typedef struct emergency {
    float time;
    int16_t timeout;
} emergency_t;
#define LNG_EMERGENCY sizeof(emergency_t)

typedef struct constraint {
    float max_left;
    float max_right;
} constraint_t;
#define LNG_CONSTRAINT sizeof(motor_t)

typedef struct motor {
    int16_t rifer_vel;
    int16_t control_vel;
    int16_t measure_vel;
    int16_t current;
} motor_t;
#define LNG_MOTOR sizeof(motor_t)

typedef struct pid {
    float kp;
    float ki;
    float kd;
} pid_control_t;
#define LNG_PID_CONTROL sizeof(pid_control_t)

typedef struct coordinate {
    float x;
    float y;
    float theta;
    float space;
} coordinate_t;
#define LNG_COORDINATE sizeof(coordinate_t)

typedef struct delta_odometry {
    coordinate_t delta[BUFFER_ODOMETRY];
} delta_odometry_t;
#define LNG_DELTA_ODOMETRY sizeof(delta_odometry_t)

typedef struct parameter_motors {
    float radius_r;
    float radius_l;
    float wheelbase;
    float k_vel_r;
    float k_vel_l;
    float k_ang_r;
    float k_ang_l;
    float sp_min;
    int16_t pwm_step;
} parameter_motors_t;
#define LNG_PARAMETER_MOTORS sizeof(parameter_motors_t)

typedef struct velocity {
    float v;
    float w;
} velocity_t;
#define LNG_VELOCITY sizeof(velocity_t)

typedef uint8_t enable_motor_t;
#define LNG_ENABLE_MOTOR sizeof(enable_motor_t)

#define ABSTRACT_PACKET_MOTION                  \
        pid_control_t pid;                      \
        coordinate_t coordinate;                \
        parameter_motors_t parameter_motors;    \
        velocity_t velocity;                    \
        enable_motor_t enable;                  \
        motor_t motor;                          \
        constraint_t constraint;                \
        emergency_t emergency;                  \
        delta_odometry_t delta_odometry;

#define PID_CONTROL_L 0
#define PID_CONTROL_R 1
#define MOTOR_L 2
#define MOTOR_R 3
#define COORDINATE 4
#define PARAMETER_MOTORS 5
#define CONSTRAINT 6
#define VELOCITY 7
#define VELOCITY_MIS 8
#define ENABLE 9
#define EMERGENCY 10
#define DELTA_ODOMETRY 11

#define PROCESS_MOTION_LENGTH 4
#define PROCESS_PID_LEFT 0
#define PID_LEFT_STRING "PID/Left"
#define PROCESS_PID_RIGHT 1
#define PID_RIGHT_STRING "PID/Right"
#define PROCESS_VELOCITY 2
#define VELOCITY_STRING "Velocity"
#define PROCESS_ODOMETRY 3
#define ODOMETRY_STRING "Odometry"

#define HASHMAP_MOTION 'M'
#define HASHMAP_MOTION_NUMBER 15
//static unsigned int hashmap_motion[HASHMAP_MOTION_NUMBER];

#define INITIALIZE_HASHMAP_MOTION hashmap_motion[PID_CONTROL_L] = LNG_PID_CONTROL;          \
                                  hashmap_motion[PID_CONTROL_R] = LNG_PID_CONTROL;          \
                                  hashmap_motion[MOTOR_L] = LNG_MOTOR;                      \
                                  hashmap_motion[MOTOR_R] = LNG_MOTOR;                      \
                                  hashmap_motion[COORDINATE] = LNG_COORDINATE;              \
                                  hashmap_motion[PARAMETER_MOTORS] = LNG_PARAMETER_MOTORS;  \
                                  hashmap_motion[CONSTRAINT] = LNG_CONSTRAINT;              \
                                  hashmap_motion[VELOCITY] = LNG_VELOCITY;                  \
                                  hashmap_motion[VELOCITY_MIS] = LNG_VELOCITY;              \
                                  hashmap_motion[ENABLE] = LNG_ENABLE_MOTOR;                \
                                  hashmap_motion[EMERGENCY] = LNG_EMERGENCY;                \
                                  hashmap_motion[DELTA_ODOMETRY] = LNG_DELTA_ODOMETRY;      \

#endif	/* MOTION_H */

