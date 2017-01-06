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
#include "high_control/manager.h"

/*****************************************************************************/
/* Parsing functions                                                         */
/*****************************************************************************/

//packet_information_t save_frame_motion(unsigned char option, unsigned char type, unsigned char command, message_abstract_u message) {
//    switch (command) {
//        case DIFF_DRIVE_COORDINATE:
//            update_motion_coordinate(message.diff_drive.coordinate);
//            break;
//        case DIFF_DRIVE_PARAMETER_UNICYCLE:
//            update_motion_parameter_unicycle(message.diff_drive.parameter_unicycle);
//            break;
//        case DIFF_DRIVE_VEL_REF:
//            set_motion_velocity_ref_unicycle(message.diff_drive.velocity);
//            break;
//        case DIFF_DRIVE_STATE:
//            set_motion_state(message.diff_drive.state);
//            break;
//        default:
//            return CREATE_PACKET_NACK(command, type);
//            break;
//    }
//    return CREATE_PACKET_ACK(command, type);
//}
//
//packet_information_t send_frame_motion(unsigned char option, unsigned char type, unsigned char command, message_abstract_u message) {
//    message_abstract_u send;
//    switch (command) {
//        case DIFF_DRIVE_COORDINATE:
//            send.diff_drive.coordinate = get_motion_coordinate();
//            break;
//        case DIFF_DRIVE_VEL_REF:
//            send.diff_drive.velocity = get_motion_velocity_ref_unicycle();
//            break;
//        case DIFF_DRIVE_VEL:
//            send.diff_drive.velocity = get_motion_velocity_meas_unicycle();
//            break;
//        case DIFF_DRIVE_STATE:
//            send.diff_drive.state = get_motion_state();
//            break;
//        case DIFF_DRIVE_PARAMETER_UNICYCLE:
//            send.diff_drive.parameter_unicycle = get_motion_parameter_unicycle();
//            break;
//        default:
//            return CREATE_PACKET_NACK(command, type);
//            break;
//    }
//    return CREATE_PACKET_DATA(command, type, send);
//}