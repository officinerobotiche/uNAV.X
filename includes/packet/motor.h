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

#ifndef MOTOR_H
#define	MOTOR_H

#include <stdint.h>

/**
 * Define to select state of control for single motor
 */
#define STATE_CONTROL_EMERGENCY -1
#define STATE_CONTROL_DISABLE 0
#define STATE_CONTROL_DIRECT 1
#define STATE_CONTROL_POSITION 2
#define STATE_CONTROL_VELOCITY 3
#define STATE_CONTROL_TORQUE 4

typedef struct _hash_motor {
    uint8_t hash_motor : 4;
    uint8_t number : 4;
} hash_motor_t;

typedef union _hash_map {
    hash_motor_t bitset;
    uint8_t hash;
} _hash_map_t;

/**
 * Message for emergency configuration
 * - time to stop
 * - time to disable bridge
 * - timeout to start emergency stop motors
 */
typedef struct emergency {
    float slope_time;
    float bridge_off;
    int16_t timeout;
} emergency_t;
#define LNG_EMERGENCY sizeof(emergency_t)

/**
 * Message to control single motor
 * - dimension number motors
 */
typedef int16_t motor_control_t;
#define LNG_MOTOR_CONTROL sizeof(motor_control_t)

/**
 * Message for state motor controller, information about:
 * - state motor - type of control
 * - mean voltage applied in the bridge - PWM
 * - torque
 * - velocity
 * - position
 */
typedef struct motor {
    int8_t state;
    motor_control_t volt;
    motor_control_t torque;
    motor_control_t velocity;
    float position;
} motor_t;
#define LNG_MOTOR sizeof(motor_t)

/**
 * Message for definition gain for PID controller
 * - K_p
 * - K_i
 * - K_d
 */
typedef struct pid {
    float kp;
    float ki;
    float kd;
} pid_control_t;
#define LNG_PID_CONTROL sizeof(pid_control_t)

/**
 * Parameter definition for motor:
 * - cpr
 * - ratio
 * - Set or
 * - boolean set enable
 */
typedef struct parameter_motor {
    float cpr;
    float ratio;
    float volt_bridge;
    int8_t encoder_pos;
    int8_t versus;
    uint8_t enable_set;
} parameter_motor_t;
#define LNG_PARAMETER_MOTOR sizeof(parameter_motor_t)

//List of all motor messages
#define ABSTRACT_MESSAGE_MOTOR                   \
        pid_control_t pid;                       \
        parameter_motor_t parameter_motor;       \
        motor_control_t motor_control;           \
        motor_t motor;                           \
        emergency_t emergency;

//Numbers associated for motor messages
#define PID_CONTROL_L 0
#define MOTOR_L 1
#define PARAMETER_MOTOR_L 2
#define VEL_MOTOR_L 3
#define VEL_MOTOR_MIS_L 4
#define ENABLE_MOTOR_L 5
#define POS_MOTOR_MIS_L 6
#define CONSTRAINT_L 7
#define EMERGENCY_L 8

#define PID_CONTROL_R 9
#define MOTOR_R 10
#define PARAMETER_MOTOR_R 11
#define VEL_MOTOR_R 12
#define VEL_MOTOR_MIS_R 13
#define ENABLE_MOTOR_R 14
#define POS_MOTOR_MIS_R 15
#define CONSTRAINT_R 16
#define EMERGENCY_R 17

//Numbers and names associated at all processes
#define PROCESS_MOTOR_LENGTH 2
#define PROCESS_PID_LEFT 0
#define PID_LEFT_STRING "PID/Left"
#define PROCESS_PID_RIGHT 1
#define PID_RIGHT_STRING "PID/Right"

//Name for HASHMAP with information about motion messages
#define HASHMAP_MOTOR 'M'
#define HASHMAP_MOTOR_NUMBER 20

// Definition on communication/parsing_packet.c
//static unsigned int hashmap_motion[HASHMAP_MOTION_NUMBER];

/**
 * Table with conversion number message in a length for data messages
 */
#define INITIALIZE_HASHMAP_MOTOR    hashmap_motion[PARAMETER_MOTOR_L] = LNG_PARAMETER_MOTOR;       \
                                    hashmap_motion[PARAMETER_MOTOR_R] = LNG_PARAMETER_MOTOR;       \
                                    hashmap_motion[PID_CONTROL_L] = LNG_PID_CONTROL;               \
                                    hashmap_motion[PID_CONTROL_R] = LNG_PID_CONTROL;               \
                                    hashmap_motion[MOTOR_L] = LNG_MOTOR;                           \
                                    hashmap_motion[MOTOR_R] = LNG_MOTOR;                           \
                                    hashmap_motion[VEL_MOTOR_L] = LNG_MOTOR_CONTROL;               \
                                    hashmap_motion[VEL_MOTOR_R] = LNG_MOTOR_CONTROL;               \
                                    hashmap_motion[VEL_MOTOR_MIS_L] = LNG_MOTOR_CONTROL;           \
                                    hashmap_motion[VEL_MOTOR_MIS_R] = LNG_MOTOR_CONTROL;           \
                                    hashmap_motion[ENABLE_MOTOR_L] = LNG_MOTOR_CONTROL;            \
                                    hashmap_motion[ENABLE_MOTOR_R] = LNG_MOTOR_CONTROL;            \
                                    hashmap_motion[POS_MOTOR_MIS_L] = LNG_MOTOR_CONTROL;           \
                                    hashmap_motion[POS_MOTOR_MIS_R] = LNG_MOTOR_CONTROL;           \
                                    hashmap_motion[CONSTRAINT_L] = LNG_MOTOR;                      \
                                    hashmap_motion[CONSTRAINT_R] = LNG_MOTOR;                      \
                                    hashmap_motion[EMERGENCY_L] = LNG_EMERGENCY;                   \
                                    hashmap_motion[EMERGENCY_R] = LNG_EMERGENCY;

#endif	/* MOTOR_H */

