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
            update_motor_pid((short) motor.bitset.motor, info->message.motor_pid);
            list_send[(*len)++] = createPacket(info->command, PACKET_ACK, info->type, NULL);
            break;
        case MOTOR_PARAMETER:
            update_motor_parameters((short) motor.bitset.motor, info->message.motor_parameter);
            list_send[(*len)++] = createPacket(info->command, PACKET_ACK, info->type, NULL);
            break;
        case MOTOR_CONSTRAINT:
            update_motor_constraints((short) motor.bitset.motor, info->message.motor);
            list_send[(*len)++] = createPacket(info->command, PACKET_ACK, info->type, NULL);
            break;
        case MOTOR_VEL_REF:
            set_motor_velocity((short) motor.bitset.motor, info->message.motor_control);
            list_send[(*len)++] = createPacket(info->command, PACKET_ACK, info->type, NULL);
            break;
        case MOTOR_STATE:
            set_motor_state((short) motor.bitset.motor, info->message.motor_state);
            list_send[(*len)++] = createPacket(info->command, PACKET_ACK, info->type, NULL);
            break;
        case MOTOR_POS_RESET:
            reset_motor_position_measure((short) motor.bitset.motor, info->message.motor_control);
            list_send[(*len)++] = createPacket(info->command, PACKET_ACK, info->type, NULL);
            break;
        case MOTOR_EMERGENCY:
            update_motor_emergency((short) motor.bitset.motor, info->message.motor_emergency);
            list_send[(*len)++] = createPacket(info->command, PACKET_ACK, info->type, NULL);
            break;
        default:
            list_send[(*len)++] = createPacket(info->command, PACKET_NACK, info->type, NULL);
            break;
    }
}

void send_frame_motor(packet_information_t* list_send, size_t* len, packet_information_t* info) {
    message_abstract_u send;
    motor.command_message = info->command;
    switch (motor.bitset.command) {
        case MOTOR_PARAMETER:
            send.motor_parameter = get_motor_parameters((short) motor.bitset.motor);
            list_send[(*len)++] = createDataPacket(info->command, info->type, &send);
            break;
        case MOTOR_VEL_PID:
            send.motor_pid = get_motor_pid((short) motor.bitset.motor);
            list_send[(*len)++] = createDataPacket(info->command, info->type, &send);
            break;
        case MOTOR_VEL_REF:
            send.motor_control = get_motor_reference((short) motor.bitset.motor).velocity;
            list_send[(*len)++] = createDataPacket(info->command, info->type, &send);
            break;
        case MOTOR_STATE:
            send.motor_state = get_motor_state((short) motor.bitset.motor);
            list_send[(*len)++] = createDataPacket(info->command, info->type, &send);
            break;
        case MOTOR:
            send.motor = get_motor_measures((short) motor.bitset.motor);
            list_send[(*len)++] = createDataPacket(info->command, info->type, &send);
            break;
        case MOTOR_CONSTRAINT:
            send.motor = get_motor_constraints((short) motor.bitset.motor);
            list_send[(*len)++] = createDataPacket(info->command, info->type, &send);
            break;
        case MOTOR_EMERGENCY:
            send.motor_emergency = get_motor_emergency((short) motor.bitset.motor);
            list_send[(*len)++] = createDataPacket(info->command, info->type, &send);
            break;
        default:
            list_send[(*len)++] = createPacket(info->command, PACKET_NACK, info->type, NULL);
            break;
    }
}