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
#include "communication/parsing_other_messages.h"
#include "communication/serial.h"

#include "system/user.h"
#include "system/system.h"

//Table to convertion name (number) of message in a length
//See packet/packet.h and packet/unav.h
static unsigned int hashmap_system[HASHMAP_SYSTEM_NUMBER];
static unsigned int hashmap_motor[HASHMAP_MOTOR_NUMBER];
static unsigned int hashmap_motion[HASHMAP_MOTION_NUMBER];

/** GLOBAL VARIBLES */
// From system/system.c
extern system_parameter_t parameter_system;
// From communication/serial.c
extern system_error_serial_t serial_error;
extern packet_t receive_pkg;
extern char receive_header;

/******************************************************************************/
/* Parsing functions                                                          */
/******************************************************************************/

void init_hashmap() {
    HASHMAP_SYSTEM_INITIALIZE
    HASHMAP_MOTOR_INITIALIZE
    INITIALIZE_HASHMAP_MOTION
}

void saveData(packet_information_t* list_send, size_t len, packet_information_t* info) {
    message_abstract_u send;
    if (info->type == HASHMAP_SYSTEM) {
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
    } else saveOtherData(list_send, len, info);
}

void sendData(packet_information_t* list_send, size_t len, packet_information_t* info) {
    message_abstract_u send;
    if (info->type == HASHMAP_SYSTEM) {
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
    } else sendOtherData(list_send, len, info);
}

int parse_packet() {
    int i;
    unsigned int t = TMR1; // Timing function
    packet_information_t list_data[BUFFER_LIST_PARSING];
    unsigned int counter = 0;
    //Save single packet
    for (i = 0; i < receive_pkg.length; i += receive_pkg.buffer[i]) {
        memcpy((unsigned char*) &list_data[counter++], &receive_pkg.buffer[i], receive_pkg.buffer[i]);
    }
    //Compute packet
    for (i = 0; i < counter; ++i) {
        packet_information_t* info = &list_data[i];
        switch (info->option) {
            case PACKET_DATA:
                saveData(&list_data[0], i, info);
                break;
            case PACKET_REQUEST:
                sendData(&list_data[0], i, info);
                break;
        }
    }
    //Send new packet
    packet_t send = encoder(&list_data[0], counter);
    if (send.length != 0)
        pkg_send(receive_header, send);
    return TMR1 - t; // Time of esecution
}

packet_t encoder(packet_information_t *list_send, size_t len) {
    int i;
    packet_t packet_send;
    packet_send.length = 0;
    for (i = 0; i < len; ++i) {
        packet_buffer_u buffer_packet;
        buffer_packet.packet_information = list_send[i];

        memcpy(&packet_send.buffer[packet_send.length], &buffer_packet.buffer, buffer_packet.packet_information.length);

        packet_send.length += buffer_packet.packet_information.length;
    }
    return packet_send;
}

packet_t encoderSingle(packet_information_t send) {
    packet_t packet_send;
    packet_send.length = send.length + 1;
    packet_buffer_u buffer_packet;
    buffer_packet.packet_information = send;
    memcpy(&packet_send.buffer, &buffer_packet.buffer, buffer_packet.packet_information.length + 1);
    return packet_send;
}

packet_information_t createPacket(unsigned char command, unsigned char option, unsigned char type, message_abstract_u * packet) {
    packet_information_t information;
    information.command = command;
    information.option = option;
    information.type = type;
    motor_command_map_t command_motor;
    if (option == PACKET_DATA) {
        switch (type) {
            case HASHMAP_SYSTEM:
                information.length = LNG_HEAD_INFORMATION_PACKET + hashmap_system[command];
                break;
            case HASHMAP_MOTION:
                information.length = LNG_HEAD_INFORMATION_PACKET + hashmap_motion[command];
                break;
            case HASHMAP_MOTOR:
                command_motor.command_message = command;
                information.length = LNG_HEAD_INFORMATION_PACKET + hashmap_motor[command_motor.bitset.command];
                break;
            default:
                //TODO throw
                break;
        }
    } else {
        information.length = LNG_HEAD_INFORMATION_PACKET;
    }
    if (packet != NULL) {
        memcpy(&information.message, packet, sizeof (message_abstract_u));
    }
    return information;
}

packet_information_t createDataPacket(unsigned char command, unsigned char type, message_abstract_u * packet) {
    return createPacket(command, PACKET_DATA, type, packet);
}