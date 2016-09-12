/*
 * Copyright (C) 2015 Officine Robotiche
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

#include <xc.h>            /* Device header file */

#include <stdint.h>        /* Includes uint16_t definition   */
#include <stdbool.h>       /* Includes true/false definition */
#include <string.h>

#include "motors/motor_comm.h"

#include "communication/serial.h"
#include <serial/or_frame.h>
#include "motors/motor_control.h"

#include "high_control/manager.h"

//Split motor and command
motor_command_map_t motor;

/******************************************************************************/
/* Parsing functions                                                          */
/******************************************************************************/

void save_frame_motor(packet_information_t* list_send, size_t* len, packet_information_t* info) {
    motor.command_message = info->command;
    switch (motor.bitset.command) {
        case MOTOR_VEL_PID:
            update_motor_pid((short) motor.bitset.motor, info->message.motor.pid);
            list_send[(*len)++] = CREATE_PACKET_ACK(info->command, info->type);
            break;
        case MOTOR_PARAMETER:
            update_motor_parameters((short) motor.bitset.motor, info->message.motor.parameter);
            list_send[(*len)++] = CREATE_PACKET_ACK(info->command, info->type);
            break;
        case MOTOR_CONSTRAINT:
            update_motor_constraints((short) motor.bitset.motor, info->message.motor.motor);
            list_send[(*len)++] = CREATE_PACKET_ACK(info->command, info->type);
            break;
        case MOTOR_VEL_REF:
            set_motor_reference((short) motor.bitset.motor, CONTROL_VELOCITY, info->message.motor.reference);
            list_send[(*len)++] = CREATE_PACKET_ACK(info->command, info->type);
            break;
        case MOTOR_STATE:
            set_motor_state((short) motor.bitset.motor, info->message.motor.state);
            list_send[(*len)++] = CREATE_PACKET_ACK(info->command, info->type);
            break;
        case MOTOR_POS_RESET:
            reset_motor_position_measure((short) motor.bitset.motor, info->message.motor.reference);
            list_send[(*len)++] = CREATE_PACKET_ACK(info->command, info->type);
            break;
        case MOTOR_EMERGENCY:
            update_motor_emergency((short) motor.bitset.motor, info->message.motor.emergency);
            list_send[(*len)++] = CREATE_PACKET_ACK(info->command, info->type);
            break;
        default:
            list_send[(*len)++] = CREATE_PACKET_NACK(info->command, info->type);
            break;
    }
}

void send_frame_motor(packet_information_t* list_send, size_t* len, packet_information_t* info) {
    message_abstract_u send;
    motor.command_message = info->command;
    switch (motor.bitset.command) {
        case MOTOR_PARAMETER:
            send.motor.parameter = get_motor_parameters((short) motor.bitset.motor);
            list_send[(*len)++] = CREATE_PACKET_DATA(info->command, info->type, send);
            break;
        case MOTOR_VEL_PID:
            send.motor.pid = get_motor_pid((short) motor.bitset.motor);
            list_send[(*len)++] = CREATE_PACKET_DATA(info->command, info->type, send);
            break;
        case MOTOR_VEL_REF:
            send.motor.reference = get_motor_reference((short) motor.bitset.motor).velocity;
            list_send[(*len)++] = CREATE_PACKET_DATA(info->command, info->type, send);
            break;
        case MOTOR_STATE:
            send.motor.state = get_motor_state((short) motor.bitset.motor);
            list_send[(*len)++] = CREATE_PACKET_DATA(info->command, info->type, send);
            break;
        case MOTOR_MEASURE:
            send.motor.motor = get_motor_measures((short) motor.bitset.motor);
            list_send[(*len)++] = CREATE_PACKET_DATA(info->command, info->type, send);
            break;
        case MOTOR_CONSTRAINT:
            send.motor.motor = get_motor_constraints((short) motor.bitset.motor);
            list_send[(*len)++] = CREATE_PACKET_DATA(info->command, info->type, send);
            break;
        case MOTOR_EMERGENCY:
            send.motor.emergency = get_motor_emergency((short) motor.bitset.motor);
            list_send[(*len)++] = CREATE_PACKET_DATA(info->command, info->type, send);
            break;
        case MOTOR_DIAGNOSTIC:
            send.motor.diagnostic = get_motor_diagnostic((short) motor.bitset.motor);
            list_send[(*len)++] = CREATE_PACKET_DATA(info->command, info->type, send);
            break;
        default:
            list_send[(*len)++] = CREATE_PACKET_NACK(info->command, info->type);
            break;
    }
}