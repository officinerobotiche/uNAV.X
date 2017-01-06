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
#include <or_bus/frame.h>
#include "motors/motor_control.h"

#include "high_control/manager.h"

//Split motor and command
motor_command_map_t motor;

/******************************************************************************/
/* Parsing functions                                                          */
/******************************************************************************/

//packet_information_t save_frame_motor(unsigned char option, unsigned char type, unsigned char command, message_abstract_u message) {
//    motor.command_message = command;
//    switch (motor.bitset.command) {
//        case MOTOR_PARAMETER:
//            update_motor_parameters((short) motor.bitset.motor, message.motor.parameter);
//            break;
//        case MOTOR_CONSTRAINT:
//            update_motor_constraints((short) motor.bitset.motor, message.motor.motor);
//            break;
//        case MOTOR_EMERGENCY:
//            update_motor_emergency((short) motor.bitset.motor, message.motor.emergency);
//            break;
//        case MOTOR_STATE:
//            set_motor_state((short) motor.bitset.motor, message.motor.state);
//            break;
//        case MOTOR_POS_RESET:
//            reset_motor_position_measure((short) motor.bitset.motor, message.motor.reference);
//            break;
////        case MOTOR_POS_REF:
////            
////            break;
//        case MOTOR_POS_PID:
//            // If the PID is not true return a NACK otherwhise return ACK
//            if( ! update_motor_pid((short) motor.bitset.motor, CONTROL_POSITION, message.motor.pid))
//                return CREATE_PACKET_NACK(command, type);
//            break;
//        case MOTOR_VEL_REF:
//            set_motor_reference((short) motor.bitset.motor, CONTROL_VELOCITY, message.motor.reference);
//            break;
//        case MOTOR_VEL_PID:
//            // If the PID is not true return a NACK otherwhise return ACK
//            if( ! update_motor_pid((short) motor.bitset.motor, CONTROL_VELOCITY, message.motor.pid))
//                return CREATE_PACKET_NACK(command, type);
//            break;
//        case MOTOR_CURRENT_REF:
//            set_motor_reference((short) motor.bitset.motor, CONTROL_CURRENT, message.motor.reference);
//            break;
//        case MOTOR_CURRENT_PID:
//            // If the PID is not true return a NACK otherwhise return ACK
//            if( ! update_motor_pid((short) motor.bitset.motor, CONTROL_CURRENT, message.motor.pid))
//                return CREATE_PACKET_NACK(command, type);
//            break;
//        case MOTOR_SAFETY:
//            update_motor_safety((short) motor.bitset.motor, message.motor.safety);
//            break;
//        default:
//            return CREATE_PACKET_NACK(command, type);
//            break;
//    }
//    return CREATE_PACKET_ACK(command, type);
//}
//
//packet_information_t send_frame_motor(unsigned char option, unsigned char type, unsigned char command, message_abstract_u message) {
//    message_abstract_u send;
//    motor.command_message = command;
//    switch (motor.bitset.command) {
//        case MOTOR_MEASURE:
//            send.motor.motor = get_motor_measures((short) motor.bitset.motor);
//            break;
//        case MOTOR_REFERENCE:
//            send.motor.motor = get_motor_reference((short) motor.bitset.motor);
//            break;
//        case MOTOR_CONTROL:
//            send.motor.motor = get_motor_control((short) motor.bitset.motor);
//            break;
//        case MOTOR_DIAGNOSTIC:
//            send.motor.diagnostic = get_motor_diagnostic((short) motor.bitset.motor);
//            break;
//        case MOTOR_PARAMETER:
//            send.motor.parameter = get_motor_parameters((short) motor.bitset.motor);
//            break;
//        case MOTOR_CONSTRAINT:
//            send.motor.motor = get_motor_constraints((short) motor.bitset.motor);
//            break;
//        case MOTOR_EMERGENCY:
//            send.motor.emergency = get_motor_emergency((short) motor.bitset.motor);
//            break;
//        case MOTOR_STATE:
//            send.motor.state = get_motor_state((short) motor.bitset.motor);
//            break;
//        case MOTOR_POS_PID:
//            send.motor.pid = get_motor_pid((short) motor.bitset.motor, CONTROL_POSITION);
//            break;
//        case MOTOR_VEL_PID:
//            send.motor.pid = get_motor_pid((short) motor.bitset.motor, CONTROL_VELOCITY);
//            break;
//        case MOTOR_CURRENT_PID:
//            send.motor.pid = get_motor_pid((short) motor.bitset.motor, CONTROL_CURRENT);
//            break;
//        case MOTOR_SAFETY:
//            send.motor.safety = get_motor_safety((short) motor.bitset.motor);
//            break;
//        default:
//            return CREATE_PACKET_NACK(command, type);
//            break;
//    }
//    return CREATE_PACKET_DATA(command, type, send);
//}
