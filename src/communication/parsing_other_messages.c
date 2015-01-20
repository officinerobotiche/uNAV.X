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

/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

/* Device header file */
#if defined(__XC16__)
#include <xc.h>
#elif defined(__C30__)
#if defined(__dsPIC33E__)
#include <p33Exxxx.h>
#elif defined(__dsPIC33F__)
#include <p33Fxxxx.h>
#endif
#endif

#include <stdint.h>        /* Includes uint16_t definition   */
#include <stdbool.h>       /* Includes true/false definition */
#include <string.h>

#include "communication/parsing_messages.h"
#include "communication/serial.h"

#include "control/high_level_control.h"
#include "control/motors_PID.h"

#include "system/user.h"
#include "system/system.h"

motor_control_t motor_temp;
abstract_message_u send_temp;

// From motors PID
extern parameter_motor_t parameter_motor_left, parameter_motor_right;
extern constraint_t constraint;
extern pid_control_t pid_left, pid_right;
extern motor_control_t motor_ref[NUM_MOTORS];
extern motor_control_t motor_state[NUM_MOTORS];
extern motor_t motor_left, motor_right;
extern emergency_t emergency;

// From high level control
extern state_controller_t control_state;
extern parameter_unicycle_t parameter_unicycle;
extern coordinate_t coordinate;
extern velocity_t vel_rif, vel_mis;
//extern delta_odometry_t delta_odometry;
extern bool coord_busy;

/******************************************************************************/
/* Computation functions                                                      */

/******************************************************************************/

void saveOtherData(information_packet_t* list_send, size_t len, information_packet_t* info) {
    if (info->type == HASHMAP_MOTION)
        switch (info->command) {
            case PID_CONTROL_L:
                pid_left = info->packet.pid;
                update_pid_l();
                list_send[len] = createPacket(info->command, ACK, info->type, NULL);
                break;
            case PID_CONTROL_R:
                pid_right = info->packet.pid;
                update_pid_r();
                list_send[len] = createPacket(info->command, ACK, info->type, NULL);
                break;
            case COORDINATE:
                coordinate = info->packet.coordinate;
                update_coord();
                list_send[len] = createPacket(info->command, ACK, info->type, NULL);
                break;
            case PARAMETER_UNICYCLE:
                parameter_unicycle = info->packet.parameter_unicycle;
                update_parameter_unicycle();
                list_send[len] = createPacket(info->command, ACK, info->type, NULL);
                break;
            case PARAMETER_MOTOR_L:
                parameter_motor_left = info->packet.parameter_motor;
                update_parameter_motors();
                list_send[len] = createPacket(info->command, ACK, info->type, NULL);
                break;
            case PARAMETER_MOTOR_R:
                parameter_motor_right = info->packet.parameter_motor;
                update_parameter_motors();
                list_send[len] = createPacket(info->command, ACK, info->type, NULL);
                break;
            case CONSTRAINT:
                constraint = info->packet.constraint;
                list_send[len] = createPacket(info->command, ACK, info->type, NULL);
                break;
            case VELOCITY:
                vel_rif = info->packet.velocity;
                UpdateHighStateController(STATE_CONTROL_HIGH_VELOCITY);
                list_send[len] = createPacket(info->command, ACK, info->type, NULL);
                break;
            case VEL_MOTOR_L:
                motor_ref[0] = info->packet.motor_control;
                UpdateStateController(0, STATE_CONTROL_VELOCITY);
                list_send[len] = createPacket(info->command, ACK, info->type, NULL);
                break;
            case VEL_MOTOR_R:
                motor_ref[1] = info->packet.motor_control;
                UpdateStateController(1, STATE_CONTROL_VELOCITY);
                list_send[len] = createPacket(info->command, ACK, info->type, NULL);
                break;
            case ENABLE_MOTOR_L:
                UpdateStateController(0, info->packet.motor_control);
                list_send[len] = createPacket(info->command, ACK, info->type, NULL);
                break;
            case ENABLE_MOTOR_R:
                UpdateStateController(1, info->packet.motor_control);
                list_send[len] = createPacket(info->command, ACK, info->type, NULL);
                break;
            case ENABLE:
                control_state = info->packet.enable;
                list_send[len] = createPacket(info->command, ACK, info->type, NULL);
                break;
            case EMERGENCY:
                emergency = info->packet.emergency;
                list_send[len] = createPacket(info->command, ACK, info->type, NULL);
                break;
            case DELTA_ODOMETRY:
            case VEL_MOTOR_MIS_L:
            case VEL_MOTOR_MIS_R:
            case MOTOR_L:
            case MOTOR_R:
            case VELOCITY_MIS:
                list_send[len] = createPacket(info->command, NACK, info->type, NULL);
                break;
            default:
                list_send[len] = createPacket(info->command, NACK, info->type, NULL);
                break;
        }
}

void sendOtherData(information_packet_t* list_send, size_t len, information_packet_t* info) {
    abstract_message_u send;
    if (info->type == HASHMAP_MOTION)
        switch (info->command) {
            case PID_CONTROL_L:
                send.pid = pid_left;
                list_send[len] = createDataPacket(info->command, info->type, &send);
                break;
            case PID_CONTROL_R:
                send.pid = pid_right;
                list_send[len] = createDataPacket(info->command, info->type, &send);
                break;
            case COORDINATE:
                send.coordinate = coordinate;
                list_send[len] = createDataPacket(info->command, info->type, &send);
                break;
                //case DELTA_ODOMETRY:
                //    send.delta_odometry = delta_odometry;
                //    list_send[len] = createDataPacket(info->command, info->type, &send);
                //    break;
            case PARAMETER_UNICYCLE:
                send.parameter_unicycle = parameter_unicycle;
                list_send[len] = createDataPacket(info->command, info->type, &send);
                break;
            case PARAMETER_MOTOR_L:
                send.parameter_motor = parameter_motor_left;
                list_send[len] = createDataPacket(info->command, info->type, &send);
                break;
            case PARAMETER_MOTOR_R:
                send.parameter_motor = parameter_motor_right;
                list_send[len] = createDataPacket(info->command, info->type, &send);
                break;
            case CONSTRAINT:
                send.constraint = constraint;
                list_send[len] = createDataPacket(info->command, info->type, &send);
                break;
            case VELOCITY:
                send.velocity = vel_rif;
                list_send[len] = createDataPacket(info->command, info->type, &send);
                break;
           case VEL_MOTOR_L:
                send.motor_control = motor_ref[0];
                list_send[len] = createDataPacket(info->command, info->type, &send);
                break;
           case VEL_MOTOR_R:
                send.motor_control = motor_ref[1];
                list_send[len] = createDataPacket(info->command, info->type, &send);
                break;
           case VEL_MOTOR_MIS_L:
                send.motor_control = motor_left.measure_vel;
                list_send[len] = createDataPacket(info->command, info->type, &send);
                break;
           case VEL_MOTOR_MIS_R:
                send.motor_control = motor_right.measure_vel;
                list_send[len] = createDataPacket(info->command, info->type, &send);
                break;
           case ENABLE_MOTOR_L:
                send.motor_control = motor_state[0];
                list_send[len] = createDataPacket(info->command, info->type, &send);
                break;
           case ENABLE_MOTOR_R:
                send.motor_control = motor_state[1];
                list_send[len] = createDataPacket(info->command, info->type, &send);
                break;
            case ENABLE:
                send.enable = control_state;
                list_send[len] = createDataPacket(info->command, info->type, &send);
                break;
            case MOTOR_L:
                send.motor = motor_left;
                list_send[len] = createDataPacket(info->command, info->type, &send);
                break;
            case MOTOR_R:
                send.motor = motor_right;
                list_send[len] = createDataPacket(info->command, info->type, &send);
                break;
            case VELOCITY_MIS:
                send.velocity = vel_mis;
                list_send[len] = createDataPacket(info->command, info->type, &send);
                break;
            case EMERGENCY:
                send.emergency = emergency;
                list_send[len] = createDataPacket(info->command, info->type, &send);
                break;
            default:
                list_send[len] = createPacket(info->command, NACK, info->type, NULL);
                break;
        }
}
