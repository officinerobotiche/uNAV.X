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

//+++++++++ NAVIGATION CONTROL ++++++++++++//

/**
 * Define to select high state of control
 */
#define STATE_CONTROL_HIGH_DISABLE 0
#define STATE_CONTROL_HIGH_VELOCITY 1
#define STATE_CONTROL_HIGH_CONFIGURATION 2

/**
 * Definition for coordinate robot:
 * - position [x, y, theta]
 * - space
 */
typedef struct coordinate {
    float x;
    float y;
    float theta;
    float space;
} coordinate_t;
#define LNG_COORDINATE sizeof(coordinate_t)

/**
 * Parameters definition for unicycle robot:
 * - radius (left and right)
 * - wheelbase
 * - minimal space for odometry
 */
typedef struct parameter_unicycle {
    float radius_r;
    float radius_l;
    float wheelbase;
    float sp_min;
} parameter_unicycle_t;
#define LNG_PARAMETER_UNICYCLE sizeof(parameter_unicycle_t)

/**
 * Message for read and write velocity in a unicycle robot:
 * - v = linear velocity
 * - w = angular velocity
 */
typedef struct velocity {
    float v;
    float w;
} velocity_t;
#define LNG_VELOCITY sizeof(velocity_t)

/**
 * Message for read and write state high level control
 */
typedef int8_t state_controller_t;
#define LNG_ENABLE_MOTOR sizeof(state_controller_t)

//Numbers associated for motion messages
#define COORDINATE 4
#define PARAMETER_UNICYCLE 5
#define VELOCITY 9
#define VELOCITY_MIS 10
#define ENABLE 11

//List of all motion messages
#define ABSTRACT_MESSAGE_MOTION                  \
        coordinate_t coordinate;                 \
        parameter_unicycle_t parameter_unicycle; \
        velocity_t velocity;                     \
        state_controller_t motor_state;

//++++++++ MOTOR CONTROL +++++++++++++++++++++//

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
 * Message for definiton contraint for velocity controller
 * Max velocity for:
 * - left motor
 * - right motor
 */
typedef struct constraint {
    int16_t max_left;
    int16_t max_right;
} constraint_t;
#define LNG_CONSTRAINT sizeof(constraint_t)

/**
 * Message to control single motor
 * - dimension number motors
 */
typedef int16_t motor_control_t;
#define LNG_MOTOR_CONTROL sizeof(motor_control_t)

/**
 * Message for state motor controller, information about:
 * - reference
 * - control
 * - measure
 * - motor current consumption
 */
typedef struct motor {
    motor_control_t refer_vel;      //TODO REMOVE
    //motor_control_t state;          //TODO state motor value
    motor_control_t control_vel;    //TODO Convert in Voltage applied
    motor_control_t current;        //TODO Convert in torque
    motor_control_t measure_vel;    //TODO Rename in velocity
    //motor_control_t position;       //TODO add angle position
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
 * Parameter definiton for motor:
 * - k_vel - See <a href="http://wiki.officinerobotiche.it/index.php?title=Robot_configuration_guide.html">Configure K_vel</a>
 * - k_ang - See <a href="http://wiki.officinerobotiche.it/index.php?title=Robot_configuration_guide.html">Configure K_ang</a>
 * - Set or
 * - boolean set enable
 */
typedef struct parameter_motor {
    float k_vel;
    float k_ang;
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
        constraint_t constraint;                 \
        emergency_t emergency;

//Numbers associated for motor messages
#define PID_CONTROL_L 0
#define PID_CONTROL_R 1
#define MOTOR_L 2
#define MOTOR_R 3
#define PARAMETER_MOTOR_L 6
#define PARAMETER_MOTOR_R 7
#define CONSTRAINT 8
#define EMERGENCY 12
#define VEL_MOTOR_L 13
#define VEL_MOTOR_R 14
#define VEL_MOTOR_MIS_L 15
#define VEL_MOTOR_MIS_R 16
#define ENABLE_MOTOR_L 17
#define ENABLE_MOTOR_R 18
#define POS_MOTOR_MIS_L 19
#define POS_MOTOR_MIS_R 20

//Numbers and names associated at all processes
#define PROCESS_MOTION_LENGTH 5
#define PROCESS_PID_LEFT 0
#define PID_LEFT_STRING "PID/Left"
#define PROCESS_PID_RIGHT 1
#define PID_RIGHT_STRING "PID/Right"

#define PROCESS_VELOCITY 2
#define VELOCITY_STRING "Velocity"
#define PROCESS_ODOMETRY 3
#define ODOMETRY_STRING "Odometry"
#define PROCESS_MEASURE_VEL 4
#define MEASURE_VEL_STRING "Measure vel"

//Name for HASHMAP with information about motion messages
#define HASHMAP_MOTION 'M'
#define HASHMAP_MOTION_NUMBER 30

// Definition on communication/parsing_packet.c
//static unsigned int hashmap_motion[HASHMAP_MOTION_NUMBER];

/**
 * Table with convertion number message in a length for data messages
 */
#define INITIALIZE_HASHMAP_MOTION   hashmap_motion[COORDINATE] = LNG_COORDINATE;                   \
                                    hashmap_motion[PARAMETER_UNICYCLE] = LNG_PARAMETER_UNICYCLE;   \
                                    hashmap_motion[VELOCITY] = LNG_VELOCITY;                       \
                                    hashmap_motion[ENABLE] = LNG_ENABLE_MOTOR;                     \
                                    hashmap_motion[VELOCITY_MIS] = LNG_VELOCITY;

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
                                    hashmap_motion[CONSTRAINT] = LNG_CONSTRAINT;                   \
                                    hashmap_motion[EMERGENCY] = LNG_EMERGENCY;
                                    
#endif	/* MOTION_H */

