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

#include "communication/serial.h"
#include <serial/frame.h>

//#include "communication/parsing_messages.h"
//#include "communication/parsing_other_messages.h"

#include "system/user.h"
#include "system/system.h"

#include "control/motors/motors.h"
#include "control/high_level_control.h"

motor_control_t motor_temp;
message_abstract_u send_temp;

//Split motor and command
motor_command_map_t motor;


/** GLOBAL VARIBLES */
// From system/system.c
extern system_parameter_t parameter_system;
// From communication/serial.c
extern system_error_serial_t serial_error;
extern packet_t receive_pkg;
extern char receive_header;

// From high level control
extern state_controller_t control_state;
extern parameter_unicycle_t parameter_unicycle;
extern coordinate_t coordinate;
extern velocity_t vel_rif, vel_mis;
extern bool coord_busy;

/******************************************************************************/
/* Parsing functions                                                          */
/******************************************************************************/

int parse_packet() {
    unsigned int t = TMR1; // Timing function
    packet_information_t list_data[BUFFER_LIST_PARSING];
    unsigned int counter = 0;

    parser();
    
    //Send new packet
    packet_t send = encoder(&list_data[0], counter);
    if (send.length != 0) {
        serial_send(receive_header, send);
    }
    return TMR1 - t; // Time of execution
}

void decodeSaveMessage(packet_information_t* list_send, size_t len, packet_information_t* info) {
    
}

void init_parsing_function() {
    /* Initialize hashmap packet */
    init_hashmap();
    
    set_frame_save(decodeSaveMessage);
    //set_frame_send(frame_reader save_f);
}

void save_frame_system(packet_information_t* list_send, size_t len, packet_information_t* info) {
    message_abstract_u send;
    switch (info->command) {
        case SYSTEM_SERVICE:
            send.system_service = services(info->message.system_service);
            list_send[len] = createDataPacket(info->command, info->type, &send);
            break;
        case SYSTEM_TASK_PRIORITY:
        case SYSTEM_TASK_FRQ:
            set_process(info->command, info->message.system_task);
            list_send[len] = createPacket(info->command, PACKET_ACK, info->type, NULL);
            break;
        case SYSTEM_TASK_NUM:
        case SYSTEM_TASK_NAME:
        case SYSTEM_TASK_TIME:
        case SYSTEM_PARAMETER:
        case SYSTEM_SERIAL_ERROR:
            list_send[len] = createPacket(info->command, PACKET_NACK, info->type, NULL);
            return;
        default:
            list_send[len] = createPacket(info->command, PACKET_NACK, info->type, NULL);
            break;
    }
}

void save_frame_motor(packet_information_t* list_send, size_t len, packet_information_t* info) {
    motor.command_message = info->command;
    switch (motor.bitset.command) {
        case MOTOR_VEL_PID:
            update_motor_pid((short) motor.bitset.motor, info->message.motor_pid);
            list_send[len] = createPacket(info->command, PACKET_ACK, info->type, NULL);
            break;
        case MOTOR_PARAMETER:
            update_motor_parameters((short) motor.bitset.motor, info->message.motor_parameter);
            list_send[len] = createPacket(info->command, PACKET_ACK, info->type, NULL);
            break;
        case MOTOR_CONSTRAINT:
            update_motor_constraints((short) motor.bitset.motor, info->message.motor);
            list_send[len] = createPacket(info->command, PACKET_ACK, info->type, NULL);
            break;
        case MOTOR_VEL_REF:
            set_motor_velocity((short) motor.bitset.motor, info->message.motor_control);
            list_send[len] = createPacket(info->command, PACKET_ACK, info->type, NULL);
            break;
        case MOTOR_STATE:
            set_motor_state((short) motor.bitset.motor, info->message.motor_state);
            control_state = STATE_CONTROL_HIGH_DISABLE; //TODO CORRECT
            list_send[len] = createPacket(info->command, PACKET_ACK, info->type, NULL);
            break;
        case MOTOR_POS_RESET:
            reset_motor_position_measure((short) motor.bitset.motor, info->message.motor_control);
            list_send[len] = createPacket(info->command, PACKET_ACK, info->type, NULL);
            break;
        case MOTOR_EMERGENCY:
            update_motor_emergency((short) motor.bitset.motor, info->message.motor_emergency);
            list_send[len] = createPacket(info->command, PACKET_ACK, info->type, NULL);
            break;
        case MOTOR:
        default:
            list_send[len] = createPacket(info->command, PACKET_NACK, info->type, NULL);
            break;
    }
}

void save_frame_motion(packet_information_t* list_send, size_t len, packet_information_t* info) {
    switch (info->command) {
        case COORDINATE:
            coordinate = info->message.motion_coordinate;
            update_coord();
            list_send[len] = createPacket(info->command, PACKET_ACK, info->type, NULL);
            break;
        case PARAMETER_UNICYCLE:
            parameter_unicycle = info->message.motion_parameter_unicycle;
            update_parameter_unicycle();
            list_send[len] = createPacket(info->command, PACKET_ACK, info->type, NULL);
            break;
        case VELOCITY:
            vel_rif = info->message.motion_velocity;
            list_send[len] = createPacket(info->command, PACKET_ACK, info->type, NULL);
            break;
        case VELOCITY_MIS:
            list_send[len] = createPacket(info->command, PACKET_NACK, info->type, NULL);
            break;
        case ENABLE:
            UpdateHighStateController(info->message.motion_state);
            list_send[len] = createPacket(info->command, PACKET_ACK, info->type, NULL);
            break;
        default:
            list_send[len] = createPacket(info->command, PACKET_NACK, info->type, NULL);
            break;
    }
}

void send_frame_system(packet_information_t* list_send, size_t len, packet_information_t* info) {
    message_abstract_u send;
    switch (info->command) {
        case SYSTEM_SERVICE:
            send.system_service = services(info->message.system_service);
            list_send[len] = createDataPacket(info->command, info->type, &send);
            break;
        case SYSTEM_TASK_PRIORITY:
        case SYSTEM_TASK_FRQ:
        case SYSTEM_TASK_TIME:
        case SYSTEM_TASK_NUM:
            send.system_task = get_process(info->command, info->message.system_task);
            list_send[len] = createDataPacket(info->command, info->type, &send);
            break;
        case SYSTEM_TASK_NAME:
            send.system_task_name = get_process_name(info->message.system_task_name);
            list_send[len] = createDataPacket(info->command, info->type, &send);
            break;
        case SYSTEM_PARAMETER:
            send.system_parameter = parameter_system;
            list_send[len] = createDataPacket(info->command, info->type, &send);
            break;
        case SYSTEM_SERIAL_ERROR:
            send.system_error_serial = serial_error;
            list_send[len] = createDataPacket(info->command, info->type, &send);
            break;
        default:
            list_send[len] = createPacket(info->command, PACKET_NACK, info->type, NULL);
            break;
    }
}