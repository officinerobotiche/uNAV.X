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
#define COORDINATE 0
#define PARAMETER_UNICYCLE 1
#define VELOCITY 2
#define VELOCITY_MIS 3
#define ENABLE 4

//List of all motion messages
#define ABSTRACT_MESSAGE_MOTION                  \
        coordinate_t coordinate;                 \
        parameter_unicycle_t parameter_unicycle; \
        velocity_t velocity;                     \
        state_controller_t motor_state;

//Name for HASHMAP with information about motion messages
#define HASHMAP_MOTION 'M'
#define HASHMAP_MOTION_NUMBER 10

// Definition on communication/parsing_packet.c
//static unsigned int hashmap_motion[HASHMAP_MOTION_NUMBER];

/**
 * Table with conversion number message in a length for data messages
 */
#define INITIALIZE_HASHMAP_MOTION   hashmap_motion[COORDINATE] = LNG_COORDINATE;                   \
                                    hashmap_motion[PARAMETER_UNICYCLE] = LNG_PARAMETER_UNICYCLE;   \
                                    hashmap_motion[VELOCITY] = LNG_VELOCITY;                       \
                                    hashmap_motion[ENABLE] = LNG_ENABLE_MOTOR;                     \
                                    hashmap_motion[VELOCITY_MIS] = LNG_VELOCITY;
                                    
#endif	/* MOTION_H */

