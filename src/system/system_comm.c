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

#include <xc.h>                     /* Device header file */

#include "system/system_comm.h"

#include "system/system.h"

#include <system/events.h>

/*****************************************************************************/
/* Global Variable Declaration                                               */
/*****************************************************************************/

/** GLOBAL VARIBLES */
// From system/system.c
extern system_parameter_t parameter_system;
// From communication/serial.c
extern system_error_serial_t serial_error;

/*****************************************************************************/
/* User Functions                                                            */
/*****************************************************************************/

packet_information_t save_frame_system(unsigned char option, unsigned char type, unsigned char command, message_abstract_u message) {
    message_abstract_u send;
    switch (command) {
        case SYSTEM_SERVICE:
            send.system.service = services(message.system.service);
            return CREATE_PACKET_DATA(command, type, send);
            break;
        case SYSTEM_TASK_PRIORITY:
        case SYSTEM_TASK_FRQ:
            set_process(command, message.system.task);
            return CREATE_PACKET_ACK(command, type);
            break;
        default:
            return CREATE_PACKET_NACK(command, type);
            break;
    }
}

packet_information_t send_frame_system(unsigned char option, unsigned char type, unsigned char command, message_abstract_u message) {
    message_abstract_u send;
    switch (command) {
        case SYSTEM_SERVICE:
            send.system.service = services(message.system.service);
            break;
        case SYSTEM_TASK_PRIORITY:
        case SYSTEM_TASK_FRQ:
        case SYSTEM_TASK_TIME:
            
            break;
        case SYSTEM_TASK_NUM:
            send.system.task = get_process(command, message.system.task);
            break;
        case SYSTEM_TASK_NAME:
            send.system.task_name = get_process_name(message.system.task_name);
            break;
        case SYSTEM_PARAMETER:
            send.system.parameter = parameter_system;
            break;
        case SYSTEM_SERIAL_ERROR:
            send.system.error_serial = serial_error;
            break;
        default:
            return CREATE_PACKET_NACK(command, type);
            break;
    }
    return CREATE_PACKET_DATA(command, type, send);
}