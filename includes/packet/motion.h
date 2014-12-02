/*
 * Copyright (C) 2014 Officine Robotiche
 * Author: Raffaello Bonghi
 * email:  raffaello.bonghi@officinerobotiche.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU Lesser General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#ifndef MOTION_H
#define	MOTION_H

#include <stdint.h>

/** Buffers dimension */
//Dimension for relative odometries data
#define BUFFER_ODOMETRY 100

/**
 * Message for emergency configuration
 * * time to stop
 * * timeout to start emergency stop motors
 */
typedef struct emergency {
    float time;
    int16_t timeout;
} emergency_t;
#define LNG_EMERGENCY sizeof(emergency_t)

/**
 * Message for definiton contraint for velocity controller
 * Max velocity for:
 * * left motor
 * * right motor
 */
typedef struct constraint {
    float max_left;
    float max_right;
} constraint_t;
#define LNG_CONSTRAINT sizeof(constraint_t)

/**
 * Message for state motor controller, information about:
 * * reference
 * * control
 * * measure
 * * motor current consumption
 */
typedef struct motor {
    int16_t refer_vel;
    int16_t control_vel;
    int16_t measure_vel;
    int16_t current;
} motor_t;
#define LNG_MOTOR sizeof(motor_t)

/**
 * Message for definition gain for PID controller
 * * K_p
 * * K_i
 * * K_d
 */
typedef struct pid {
    float kp;
    float ki;
    float kd;
} pid_control_t;
#define LNG_PID_CONTROL sizeof(pid_control_t)

/**
 * Definition for coordinate robot:
 * * position [x, y, theta]
 * * space
 */
typedef struct coordinate {
    float x;
    float y;
    float theta;
    float space;
} coordinate_t;
#define LNG_COORDINATE sizeof(coordinate_t)

/**
 * Definiton for relative odometry. A buffer of BUFFER_ODOMETRY size
 * with more structs coordinate_t
 */
//TODO correction this messages TOO LARGE
//typedef struct delta_odometry {
//    coordinate_t delta[BUFFER_ODOMETRY];
//} delta_odometry_t;
//#define LNG_DELTA_ODOMETRY sizeof(delta_odometry_t)

/**
 * Parameters definition for unicycle robot:
 * * radius (left and right)
 * * wheelbase
 * * k_vel (left and right)
 * * k_ang (left and right)
 * * minimal space
 */
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

/**
 * Message for read and write velocity in a unicycle robot:
 * * v = linear velocity
 * * w = angular velocity
 */
typedef struct velocity {
    float v;
    float w;
} velocity_t;
#define LNG_VELOCITY sizeof(velocity_t)

/**
 * Message for read and write state of H-bridge (able or disable)
 */
typedef uint8_t enable_motor_t;
#define LNG_ENABLE_MOTOR sizeof(enable_motor_t)

//List of all motion messages
#define ABSTRACT_MESSAGE_MOTION                  \
        pid_control_t pid;                       \
        coordinate_t coordinate;                 \
        parameter_motors_t parameter_motors;     \
        velocity_t velocity;                     \
        enable_motor_t enable;                   \
        motor_t motor;                           \
        constraint_t constraint;                 \
        emergency_t emergency;                  
        //delta_odometry_t delta_odometry;

//Numbers associated for motion messages
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

//Numbers and names associated at all processes
#define PROCESS_MOTION_LENGTH 4
#define PROCESS_PID_LEFT 0
#define PID_LEFT_STRING "PID/Left"
#define PROCESS_PID_RIGHT 1
#define PID_RIGHT_STRING "PID/Right"
#define PROCESS_VELOCITY 2
#define VELOCITY_STRING "Velocity"
#define PROCESS_ODOMETRY 3
#define ODOMETRY_STRING "Odometry"

//Name for HASHMAP with information about motion messages
#define HASHMAP_MOTION 'M'
#define HASHMAP_MOTION_NUMBER 15

// Definition on communication/parsing_packet.c
//static unsigned int hashmap_motion[HASHMAP_MOTION_NUMBER];

/**
 * Table with convertion number message in a length for data messages
 */
#define INITIALIZE_HASHMAP_MOTION   hashmap_motion[PID_CONTROL_L] = LNG_PID_CONTROL;          \
                                    hashmap_motion[PID_CONTROL_R] = LNG_PID_CONTROL;          \
                                    hashmap_motion[MOTOR_L] = LNG_MOTOR;                      \
                                    hashmap_motion[MOTOR_R] = LNG_MOTOR;                      \
                                    hashmap_motion[COORDINATE] = LNG_COORDINATE;              \
                                    hashmap_motion[PARAMETER_MOTORS] = LNG_PARAMETER_MOTORS;  \
                                    hashmap_motion[CONSTRAINT] = LNG_CONSTRAINT;              \
                                    hashmap_motion[VELOCITY] = LNG_VELOCITY;                  \
                                    hashmap_motion[VELOCITY_MIS] = LNG_VELOCITY;              \
                                    hashmap_motion[ENABLE] = LNG_ENABLE_MOTOR;                \
                                    hashmap_motion[EMERGENCY] = LNG_EMERGENCY;
                                    //hashmap_motion[DELTA_ODOMETRY] = LNG_DELTA_ODOMETRY;

#endif	/* MOTION_H */

