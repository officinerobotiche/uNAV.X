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

#include <or_system/events.h>

/*****************************************************************************/
/* Global Variable Declaration                                               */
/*****************************************************************************/

/*****************************************************************************/
/* User Functions                                                            */
/*****************************************************************************/

//packet_information_t save_frame_system(unsigned char option, unsigned char type, unsigned char command, message_abstract_u message) {
////    message_abstract_u send;
//    switch (command) {
////        case SYSTEM_TASK_PRIORITY:
////        case SYSTEM_TASK_FRQ:
////            set_process(command, message.system.task);
////            return CREATE_PACKET_ACK(command, type);
////            break;
//        default:
//            return CREATE_PACKET_NACK(command, type);
//            break;
//    }
//}
//
//packet_information_t send_frame_system(unsigned char option, unsigned char type, unsigned char command, message_abstract_u message) {
//    message_abstract_u send;
//    switch (command) {
//        case SYSTEM_RESET:
//            // The board is in reset mode. It's futile to send other message 
//            // after this function
//            reset();
//            break;
//        case SYSTEM_CODE_DATE:
//        case SYSTEM_CODE_VERSION:
//        case SYSTEM_CODE_AUTHOR:
//        case SYSTEM_CODE_BOARD_TYPE:
//        case SYSTEM_CODE_BOARD_NAME:
//            services(command, &send);
//            break;
//        case SYSTEM_TIME:
//            get_system_time(&send);
//            break;
////        case SYSTEM_SERIAL_ERROR:
////            send.system.error_serial = serial_error;
////            break;
//        default:
//            return CREATE_PACKET_NACK(command, type);
//            break;
//    }
//    return CREATE_PACKET_DATA(command, type, send);
//}
