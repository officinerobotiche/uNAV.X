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
#define STATE_CONTROL_EMERGENCY     -1  ///< Motors slow down to zero speed, then the bridge is turned off
#define STATE_CONTROL_DISABLE       0   ///< Motors disabled
#define STATE_CONTROL_POSITION      1   ///< Motors controlled in position
#define STATE_CONTROL_VELOCITY      2   ///< Motors controlled in velocity
#define STATE_CONTROL_TORQUE        3   ///< Motors controller in torque
#define STATE_CONTROL_DIRECT        4   ///< Motors controlled using direct PWM signals

/**
 * This union converts a command message in a motor index and type of command
 * - [#] maximum motor 2^4 = 16
 * - [#] maximum command 2^4 = 16
 */
typedef union _motor_command_map {

    struct {
        unsigned char motor : 4;    ///< Motor index
        unsigned char command : 4;  ///< Motor command
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
 * Message to get stateus of a single motor
 * - [#] state of control
 */
typedef int8_t motor_state_t;
#define LNG_MOTOR_STATE sizeof(motor_state_t)

/**
 * Message for the status of the motor controller, information about:
 * - [#]       state motor - type of control
 * - [mV]      mean voltage applied in the bridge - PWM
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
 * All diagnostic information about state of motor
 * - [mA]  Motor current
 * - [m°C] Temperature motor
 */
typedef struct _motor_diagnostic {
    int16_t current;
    uint16_t temperature;
} motor_diagnostic_t;
#define LNG_MOTOR_DIAGNOSTIC sizeof(motor_diagnostic_t)

/**
 * Encoder parameters definition:
 * - [ 0, 1] Position encoder respect to gear [0 after, 1 before]
 * - [#]     Encoder CPR
 * - [ 0, 1] Z-index [0 false, 1 true]
 */
#define MOTOR_GEAR_ENC_AFTER 0
#define MOTOR_GEAR_ENC_BEFORE 1
typedef struct _motor_parameter_encoder {
    uint8_t position;
    uint16_t cpr;
    uint8_t z_index;
} motor_parameter_encoder_t;
#define LNG_MOTOR_PARAMETER_ENCODER sizeof(motor_parameter_encoder_t)

/**
 * Parameters definition for motor:
 * - [mV]    Supplied voltage in H-bridge
 * - [ 0, 1] Default logic value to enable the H-bridge [0 low, 1 high]
 * - [XXX] TODO
 * - [ 0, 1] Current sensor with sign [0 false, 1 true]
 * - [XXX] TODO
 */
#define MOTOR_ENABLE_LOW 0
#define MOTOR_ENABLE_HIGH 1
typedef struct _motor_parameter_bridge {
    uint16_t volt;
    uint8_t enable;
    int16_t dead_zone;
    uint8_t signed_current;
    float k_bemf;
} motor_parameter_bridge_t;
#define LNG_MOTOR_PARAMETER_BRIDGE sizeof(motor_parameter_bridge_t)

/**
 * Collection of parameters to configure bridge and encoder
 * - Bridge configuration parameters
 * - Encoder parameters
 * - [-1, 1] Positive versus of the rotation of the motor [-1 clockwise, 1 counterclockwise]
 * - [#]     Gear ratio
 */
#define MOTOR_ROTATION_CLOCKWISE -1
#define MOTOR_ROTATION_COUNTERCLOCKWISE 1
typedef struct _motor_parameter {
    motor_parameter_bridge_t bridge;
    motor_parameter_encoder_t encoder;
    int8_t rotation;
    float ratio;
} motor_parameter_t;
#define LNG_MOTOR_PARAMETER sizeof(motor_parameter_t)

/**
 * Message for emergency configuration
 * - [s]  Time to put velocity motor to zero
 * - [s]  Time to disable bridge after the speed reaches velocity is zero
 * - [ms] Timeout to start emergency stop of the motors
 */
typedef struct _motor_emergency {
    float slope_time;
    float bridge_off;
    uint16_t timeout;
} motor_emergency_t;
#define LNG_MOTOR_EMERGENCY sizeof(motor_emergency_t)

/**
 * Message to define the gains for a PID controller
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
#define ABSTRACT_MESSAGE_MOTOR                             \
        motor_t motor;                                     \
        motor_diagnostic_t motor_diagnostic;               \
        motor_parameter_t motor_parameter;                 \
        motor_parameter_encoder_t motor_parameter_encoder; \
        motor_parameter_bridge_t motor_parameter_bridge;   \
        motor_state_t motor_state;                         \
        motor_emergency_t motor_emergency;                 \
        motor_pid_t motor_pid;                             \
        motor_control_t motor_control;

//Numbers associated for motor messages to be used in the structure @ref motor_command_map_t as value for @ref command
#define MOTOR                     0 ///< TODO Explain what this means
#define MOTOR_DIAGNOSTIC          1 ///< TODO Explain what this means
#define MOTOR_PARAMETER           2 ///< TODO Explain what this means
#define MOTOR_PARAMETER_ENCODER   3 ///< TODO Explain what this means
#define MOTOR_PARAMETER_BRIDGE    4 ///< TODO Explain what this means
#define MOTOR_CONSTRAINT          5 ///< TODO Explain what this means
#define MOTOR_EMERGENCY           6 ///< TODO Explain what this means
#define MOTOR_STATE               7 ///< TODO Explain what this means
#define MOTOR_POS_RESET           8 ///< TODO Explain what this means
#define MOTOR_POS_PID             9 ///< TODO Explain what this means
#define MOTOR_POS_REF            10 ///< TODO Explain what this means
#define MOTOR_VEL_PID            11 ///< TODO Explain what this means
#define MOTOR_VEL_REF            12 ///< TODO Explain what this means
#define MOTOR_TORQUE_PID         13 ///< TODO Explain what this means
#define MOTOR_TORQUE_REF         14 ///< TODO Explain what this means

//Name for HASHMAP with information about motion messages
#define HASHMAP_MOTOR 'G'
#define HASHMAP_MOTOR_NUMBER 16

// Definition on communication/parsing_packet.c
//static unsigned int hashmap_motor[HASHMAP_MOTOR_NUMBER];

/**
 * Table with conversion number message in a length for data messages
 */
#define HASHMAP_MOTOR_INITIALIZE    hashmap_motor[MOTOR] = LNG_MOTOR;                                     \
                                    hashmap_motor[MOTOR_DIAGNOSTIC] = LNG_MOTOR_PARAMETER;                \
                                    hashmap_motor[MOTOR_PARAMETER] = LNG_MOTOR_PARAMETER;                 \
                                    hashmap_motor[MOTOR_PARAMETER_ENCODER] = LNG_MOTOR_PARAMETER_ENCODER; \
                                    hashmap_motor[MOTOR_PARAMETER_BRIDGE] = LNG_MOTOR_PARAMETER_BRIDGE;   \
                                    hashmap_motor[MOTOR_CONSTRAINT] = LNG_MOTOR;                          \
                                    hashmap_motor[MOTOR_EMERGENCY] = LNG_MOTOR_EMERGENCY;                 \
                                    hashmap_motor[MOTOR_STATE] = LNG_MOTOR_STATE;                         \
                                    hashmap_motor[MOTOR_POS_RESET] = LNG_MOTOR_CONTROL;                   \
                                    hashmap_motor[MOTOR_POS_PID] = LNG_MOTOR_PID;                         \
                                    hashmap_motor[MOTOR_POS_REF] = LNG_MOTOR_CONTROL;                     \
                                    hashmap_motor[MOTOR_VEL_PID] = LNG_MOTOR_PID;                         \
                                    hashmap_motor[MOTOR_VEL_REF] = LNG_MOTOR_CONTROL;                     \
                                    hashmap_motor[MOTOR_TORQUE_PID] = LNG_MOTOR_PID;                      \
                                    hashmap_motor[MOTOR_TORQUE_REF] = LNG_MOTOR_CONTROL;

#endif	/* MOTOR_H */