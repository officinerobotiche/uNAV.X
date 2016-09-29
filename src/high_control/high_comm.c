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

#include "high_control/high_comm.h"

#include "communication/serial.h"
#include <or_bus/or_frame.h>
#include "high_control/manager.h"

/*****************************************************************************/
/* Parsing functions                                                         */
/*****************************************************************************/

packet_information_t save_frame_motion(unsigned char option, unsigned char type, unsigned char command, message_abstract_u message) {
    switch (command) {
        case MOTION_COORDINATE:
            update_motion_coordinate(message.motion.coordinate);
            break;
        case MOTION_PARAMETER_UNICYCLE:
            update_motion_parameter_unicycle(message.motion.parameter_unicycle);
            break;
        case MOTION_VEL_REF:
            set_motion_velocity_ref_unicycle(message.motion.velocity);
            break;
        case MOTION_STATE:
            set_motion_state(message.motion.state);
            break;
        default:
            return CREATE_PACKET_NACK(command, type);
            break;
    }
    return CREATE_PACKET_ACK(command, type);
}

packet_information_t send_frame_motion(unsigned char option, unsigned char type, unsigned char command, message_abstract_u message) {
    message_abstract_u send;
    switch (command) {
        case MOTION_COORDINATE:
            send.motion.coordinate = get_motion_coordinate();
            break;
        case MOTION_VEL_REF:
            send.motion.velocity = get_motion_velocity_ref_unicycle();
            break;
        case MOTION_VEL:
            send.motion.velocity = get_motion_velocity_meas_unicycle();
            break;
        case MOTION_STATE:
            send.motion.state = get_motion_state();
            break;
        case MOTION_PARAMETER_UNICYCLE:
            send.motion.parameter_unicycle = get_motion_parameter_unicycle();
            break;
        default:
            return CREATE_PACKET_NACK(command, type);
            break;
    }
    return CREATE_PACKET_DATA(command, type, send);
}