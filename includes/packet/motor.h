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

/**
 * This uniun convert a command message in a number motor and type of command
 * - [#] maximum motor 2^4 = 16
 * - [#] maximum command 2^4 = 16
 */
typedef union _motor_command_map {

    struct {
        unsigned char motor : 4;
        unsigned char command : 4;
    } bitset;
    unsigned char command_message;
} motor_command_map_t;

/**
 * Message to control single motor
 * - [X] command or measure to control [physic dimension depends to type of command]
 */
typedef int16_t motor_control_t;
#define LNG_MOTOR_CONTROL sizeof(motor_control_t)

/**
 * Message to state single motor
 * - [#] state of control
 */
typedef int8_t motor_state_t;
#define LNG_MOTOR_STATE sizeof(motor_state_t)

/**
 * Message for state motor controller, information about:
 * - [#]       state motor - type of control
 * - [mV]      mean voltage applied in the bridge - PWM
 * - [Nm]      torque
 * - [m rad/s] velocity
 * - [rad]     position
 * - [rad]     delta position
 */
typedef struct _motor {
    motor_state_t state;
    motor_control_t volt;
    motor_control_t torque;
    motor_control_t velocity;
    float position;
    float position_delta;
} motor_t;
#define LNG_MOTOR sizeof(motor_t)

/**
 * Parameter definition for motor:
 * - [#]     Encoder CPR
 * - [#]     Gear ratio
 * - [mV]    Supplied voltage in H-bridge
 * - [0,  1] Position encoder respect to gear [0 after, 1 before]
 * - [-1, 1] Versus rotation motor [1 counterclockwise, -1 clockwise]
 * - [0,  1] Default logic value for enable H-bridge [0 low, 1 high]
 */
typedef struct _motor_parameter {
    uint16_t cpr;
    float ratio;
    motor_control_t volt_bridge;
    uint8_t encoder_pos;
    int8_t versus;
    uint8_t enable_set;
} motor_parameter_t;
#define LNG_MOTOR_PARAMETER sizeof(motor_parameter_t)

/**
 * Message for emergency configuration
 * - [s]  Time to put velocity motor to zero
 * - [s]  Time to disable bridge
 * - [ms] Timeout to start emergency stop motors
 */
typedef struct _motor_emergency {
    float slope_time;
    float bridge_off;
    int16_t timeout;
} motor_emergency_t;
#define LNG_MOTOR_EMERGENCY sizeof(motor_emergency_t)

/**
 * Message for definition gain for PID controller
 * - [X] K_p [physic dimension depends to type of control]
 * - [X] K_i [physic dimension depends to type of control]
 * - [X] K_d [physic dimension depends to type of control]
 */
typedef struct _motor_pid {
    float kp;
    float ki;
    float kd;
} motor_pid_t;
#define LNG_MOTOR_PID sizeof(motor_pid_t)

//List of all motor messages
#define ABSTRACT_MESSAGE_MOTOR                   \
        motor_t motor;                           \
        motor_parameter_t motor_parameter;       \
        motor_state_t motor_state;               \
        motor_emergency_t motor_emergency;       \
        motor_pid_t motor_pid;                   \
        motor_control_t motor_control;

//Numbers associated for motor messages
#define MOTOR 0
#define MOTOR_PARAMETER 1
#define MOTOR_CONSTRAINT 2
#define MOTOR_EMERGENCY 3
#define MOTOR_STATE 4
#define MOTOR_VEL_PID 5
#define MOTOR_VEL_REF 6
#define MOTOR_VEL_MEAS 7
#define MOTOR_POS_MEAS 8

//Name for HASHMAP with information about motion messages
#define HASHMAP_MOTOR 'G'
#define HASHMAP_MOTOR_NUMBER 16

// Definition on communication/parsing_packet.c
//static unsigned int hashmap_motor[HASHMAP_MOTOR_NUMBER];

/**
 * Table with conversion number message in a length for data messages
 */
#define HASHMAP_MOTOR_INITIALIZE    hashmap_motor[MOTOR] = LNG_MOTOR;                           \
                                    hashmap_motor[MOTOR_PARAMETER] = LNG_MOTOR_PARAMETER;       \
                                    hashmap_motor[MOTOR_CONSTRAINT] = LNG_MOTOR;                \
                                    hashmap_motor[MOTOR_EMERGENCY] = LNG_MOTOR_EMERGENCY;       \
                                    hashmap_motor[MOTOR_STATE] = LNG_MOTOR_STATE;               \
                                    hashmap_motor[MOTOR_VEL_PID] = LNG_MOTOR_PID;               \
                                    hashmap_motor[MOTOR_VEL_REF] = LNG_MOTOR_CONTROL;           \
                                    hashmap_motor[MOTOR_VEL_MEAS] = LNG_MOTOR_CONTROL;          \
                                    hashmap_motor[MOTOR_POS_MEAS] = LNG_MOTOR_CONTROL;

#endif	/* MOTOR_H */

