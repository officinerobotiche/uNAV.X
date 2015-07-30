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

#include "high_control/high_comm.h"

#include "communication/serial.h"
#include <serial/or_frame.h>
#include "high_control/manager.h"

/*****************************************************************************/
/* Parsing functions                                                         */
/*****************************************************************************/

void save_frame_motion(packet_information_t* list_send, size_t* len, packet_information_t* info) {
    switch (info->command) {
        case MOTION_COORDINATE:
            update_motion_coordinate(info->message.motion.coordinate);
            list_send[(*len)++] = CREATE_PACKET_ACK(info->command, info->type);
            break;
        case MOTION_PARAMETER_UNICYCLE:
            update_motion_parameter_unicycle(info->message.motion.parameter_unicycle);
            list_send[(*len)++] = CREATE_PACKET_ACK(info->command, info->type);
            break;
        case MOTION_VEL_REF:
            set_motion_velocity_ref_unicycle(info->message.motion.velocity);
            list_send[(*len)++] = CREATE_PACKET_ACK(info->command, info->type);
            break;
        case MOTION_STATE:
            set_motion_state(info->message.motion.state);
            list_send[(*len)++] = CREATE_PACKET_ACK(info->command, info->type);
            break;
        default:
            list_send[(*len)++] = CREATE_PACKET_NACK(info->command, info->type);
            break;
    }
}

void send_frame_motion(packet_information_t* list_send, size_t* len, packet_information_t* info) {
    message_abstract_u send;
    switch (info->command) {
        case MOTION_COORDINATE:
            send.motion.coordinate = get_motion_coordinate();
            list_send[(*len)++] = CREATE_PACKET_DATA(info->command, info->type, send);
            break;
        case MOTION_VEL_REF:
            send.motion.velocity = get_motion_velocity_ref_unicycle();
            list_send[(*len)++] = CREATE_PACKET_DATA(info->command, info->type, send);
            break;
        case MOTION_VEL:
            send.motion.velocity = get_motion_velocity_meas_unicycle();
            list_send[(*len)++] = CREATE_PACKET_DATA(info->command, info->type, send);
            break;
        case MOTION_STATE:
            send.motion.state = get_motion_state();
            list_send[(*len)++] = CREATE_PACKET_DATA(info->command, info->type, send);
            break;
        case MOTION_PARAMETER_UNICYCLE:
            send.motion.parameter_unicycle = get_motion_parameter_unicycle();
            list_send[(*len)++] = CREATE_PACKET_DATA(info->command, info->type, send);
            break;
        default:
            list_send[(*len)++] = CREATE_PACKET_NACK(info->command, info->type);
            break;
    }
}