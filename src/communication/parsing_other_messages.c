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
#include "control/motors.h"

#include "system/user.h"
#include "system/system.h"

motor_control_t motor_temp;
abstract_message_u send_temp;

//Split motor and command
motor_command_map_t motor;

// From high level control
extern state_controller_t control_state;
extern parameter_unicycle_t parameter_unicycle;
extern coordinate_t coordinate;
extern velocity_t vel_rif, vel_mis;
extern bool coord_busy;


/*****************************************************************************/
/* Computation functions                                                     */
/*****************************************************************************/

void saveOtherData(information_packet_t* list_send, size_t len, information_packet_t* info) {
    if (info->type == HASHMAP_MOTOR) {
        motor.command_message = info->command;
        switch (motor.bitset.command) {
            case MOTOR_VEL_PID:
                update_motor_pid(motor.bitset.motor, info->packet.pid);
                list_send[len] = createPacket(info->command, ACK, info->type, NULL);
                break;
            case MOTOR_PARAMETER:
                update_motor_parameters(motor.bitset.motor, info->packet.parameter_motor);
                list_send[len] = createPacket(info->command, ACK, info->type, NULL);
                break;
            case MOTOR_CONSTRAINT:
                update_motor_constraints(motor.bitset.motor, info->packet.motor);
                list_send[len] = createPacket(info->command, ACK, info->type, NULL);
                break;
            case MOTOR_VEL_REF:
                set_motor_velocity(motor.bitset.motor, info->packet.motor_control);
                list_send[len] = createPacket(info->command, ACK, info->type, NULL);
                break;
            case MOTOR_STATE:
                set_motor_state(motor.bitset.motor, info->packet.motor_state);
                control_state = STATE_CONTROL_HIGH_DISABLE; //TODO CORRECT
                list_send[len] = createPacket(info->command, ACK, info->type, NULL);
                break;
            case MOTOR_EMERGENCY:
                update_motor_emergency(motor.bitset.motor, info->packet.emergency);
                list_send[len] = createPacket(info->command, ACK, info->type, NULL);
                break;
            case MOTOR_VEL_MEAS:
            case MOTOR_POS_MEAS:
            case MOTOR:
            default:
                list_send[len] = createPacket(info->command, NACK, info->type, NULL);
                break;
        }
    } else if (info->type == HASHMAP_MOTION) {
        switch (info->command) {
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
            case VELOCITY:
                vel_rif = info->packet.velocity;
                list_send[len] = createPacket(info->command, ACK, info->type, NULL);
                break;
            case VELOCITY_MIS:
                list_send[len] = createPacket(info->command, NACK, info->type, NULL);
                break;
            case ENABLE:
                UpdateHighStateController(info->packet.motor_state);
                list_send[len] = createPacket(info->command, ACK, info->type, NULL);
                break;
            default:
                list_send[len] = createPacket(info->command, NACK, info->type, NULL);
                break;
        }
    }
}

void sendOtherData(information_packet_t* list_send, size_t len, information_packet_t* info) {
    abstract_message_u send;
    if (info->type == HASHMAP_MOTOR) {
        motor.command_message = info->command;
        switch (motor.bitset.command) {
            case MOTOR_PARAMETER:
                send.parameter_motor = get_motor_parameters(motor.bitset.motor);
                list_send[len] = createDataPacket(info->command, info->type, &send);
                break;
            case MOTOR_VEL_PID:
                send.pid = get_motor_pid(motor.bitset.motor);
                list_send[len] = createDataPacket(info->command, info->type, &send);
                break;
           case MOTOR_VEL_REF:
                send.motor_control = get_motor_reference(motor.bitset.motor).velocity;
                list_send[len] = createDataPacket(info->command, info->type, &send);
                break;
           case MOTOR_VEL_MEAS:
                send.motor_control = get_motor_measures(motor.bitset.motor).velocity;
                list_send[len] = createDataPacket(info->command, info->type, &send);
                break;
           case MOTOR_STATE:
                send.motor_state = get_motor_state(motor.bitset.motor);
                list_send[len] = createDataPacket(info->command, info->type, &send);
                break;
            case MOTOR:
                send.motor = get_motor_measures(motor.bitset.motor);
                list_send[len] = createDataPacket(info->command, info->type, &send);
                break;
            case MOTOR_POS_MEAS:
                send.motor_control = get_motor_measures(motor.bitset.motor).position;
                list_send[len] = createDataPacket(info->command, info->type, &send);
                break;
            case MOTOR_CONSTRAINT:
                send.motor = get_motor_constraints(motor.bitset.motor);
                list_send[len] = createDataPacket(info->command, info->type, &send);
                break;
            case MOTOR_EMERGENCY:
                send.emergency = get_motor_emergency(motor.bitset.motor);
                list_send[len] = createDataPacket(info->command, info->type, &send);
                break;
            default:
                list_send[len] = createPacket(info->command, NACK, info->type, NULL);
                break;
        }
    } else if (info->type == HASHMAP_MOTION) {
        switch (info->command) {
            case COORDINATE:
                send.coordinate = coordinate;
                list_send[len] = createDataPacket(info->command, info->type, &send);
                break;
            case VELOCITY:
                send.velocity = vel_rif;
                list_send[len] = createDataPacket(info->command, info->type, &send);
                break;
            case VELOCITY_MIS:
                send.velocity = vel_mis;
                list_send[len] = createDataPacket(info->command, info->type, &send);
                break;
            case ENABLE:
                send.motor_state = control_state;
                list_send[len] = createDataPacket(info->command, info->type, &send);
                break;
            case PARAMETER_UNICYCLE:
                send.parameter_unicycle = parameter_unicycle;
                list_send[len] = createDataPacket(info->command, info->type, &send);
                break;
            default:
                list_send[len] = createPacket(info->command, NACK, info->type, NULL);
                break;
        }
    }
}
