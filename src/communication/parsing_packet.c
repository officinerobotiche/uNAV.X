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

#include "communication/parsing_packet.h"
#include "communication/decode_packet.h"
#include "communication/serial.h"

#include "system/user.h"
#include "system/system.h"

static unsigned int hashmap_default[10];
static unsigned int hashmap_motion[HASHMAP_MOTION_NUMBER];

//From System
extern parameter_system_t parameter_system;

// From Interrupt
extern volatile process_t time, priority, frequency;
extern process_buffer_t name_process_pid_l, name_process_pid_r, name_process_velocity, name_process_odometry;

// From serial
extern error_pkg_t serial_error;
extern packet_t receive_pkg;
extern char receive_header;

/******************************************************************************/
/* Parsing functions                                                          */
/******************************************************************************/

void init_hashmap() {
    INITIALIZE_HASHMAP_DEFAULT
    INITIALIZE_HASHMAP_MOTION
}

void saveData(information_packet_t* list_send, size_t len, information_packet_t info) {
    abstract_packet_t send;
    if (info.type == HASHMAP_DEFAULT) {
        switch (info.command) {
            case SERVICES:
                send.services = services(info.packet.services);
                list_send[len] = createDataPacket(info.command, info.type, &send);
                break;
            case PRIORITY_PROCESS:
                priority = info.packet.process;
                list_send[len] = createPacket(info.command, ACK, info.type, NULL);
            case FRQ_PROCESS:
                frequency = info.packet.process;
                list_send[len] = createPacket(info.command, ACK, info.type, NULL);
                break;
            case NAME_PROCESS:
                send.process_name = decodeNameProcess(info.packet.process_name.name);
                list_send[len] = createDataPacket(info.command, info.type, &send);
                break;
            case TIME_PROCESS:
            case PARAMETER_SYSTEM:
            case ERROR_SERIAL:
                list_send[len] = createPacket(info.command, NACK, info.type, NULL);
                return;
            default:
                list_send[len] = createPacket(info.command, NACK, info.type, NULL);
                break;
        }
    } else saveOtherData(list_send, len, info);
}

void sendData(information_packet_t* list_send, size_t len, information_packet_t info) {
    abstract_packet_t send;
    if (info.type == HASHMAP_DEFAULT) {
        switch (info.command) {
            case SERVICES:
                send.services = services(info.packet.services);
                list_send[len] = createDataPacket(info.command, info.type, &send);
                break;
            case PRIORITY_PROCESS:
                send.process = priority;
                list_send[len] = createDataPacket(info.command, info.type, &send);
            case FRQ_PROCESS:
                send.process = frequency;
                list_send[len] = createDataPacket(info.command, info.type, &send);
                break;
            case TIME_PROCESS:
                send.process = time;
                list_send[len] = createDataPacket(info.command, info.type, &send);
                break;
            case PARAMETER_SYSTEM:
                send.parameter_system = parameter_system;
                list_send[len] = createDataPacket(info.command, info.type, &send);
                break;
            case ERROR_SERIAL:
                send.error_pkg = serial_error;
                list_send[len] = createDataPacket(info.command, info.type, &send);
                break;
            case NAME_PROCESS:
            default:
                list_send[len] = createPacket(info.command, NACK, info.type, NULL);
                break;
        }
    } else sendOtherData(list_send, len, info);
}

/**
 * on packet:
 * -------------------------- ---------------------------- -----------------------
 * | Length | CMD | DATA ... | Length | CMD | INFORMATION |Length | CMD | ... ... |
 * -------------------------- ---------------------------- -----------------------
 *    1        2 -> length    length+1 length+2 length+3   ....
 *
 * @return
 */
int parse_packet() {
    int i;
    unsigned int t = TMR1; // Timing function
    information_packet_t list_data[10];
    unsigned int counter = 0;
    //Save single packet
    for (i = 0; i < receive_pkg.length;) {
        buffer_packet_u buffer_packet;
        memcpy(&buffer_packet.buffer, &receive_pkg.buffer[i], receive_pkg.buffer[i]);
        list_data[counter++] = buffer_packet.information_packet;
        i += receive_pkg.buffer[i];
    }
    //Compute packet
    for (i = 0; i < counter; ++i) {
        information_packet_t info = list_data[i];
        switch (info.option) {
            case DATA:
                saveData(&list_data[0], i, info);
                break;
            case REQUEST:
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

packet_t encoder(information_packet_t *list_send, size_t len) {
    int i;
    packet_t packet_send;
    packet_send.length = 0;
    for (i = 0; i < len; ++i) {
        buffer_packet_u buffer_packet;
        buffer_packet.information_packet = list_send[i];

        memcpy(&packet_send.buffer[packet_send.length], &buffer_packet.buffer, buffer_packet.information_packet.length);

        packet_send.length += buffer_packet.information_packet.length;
    }
    return packet_send;
}

packet_t encoderSingle(information_packet_t send) {
    packet_t packet_send;
    packet_send.length = send.length + 1;
    buffer_packet_u buffer_packet;
    buffer_packet.information_packet = send;
    memcpy(&packet_send.buffer, &buffer_packet.buffer, buffer_packet.information_packet.length + 1);
    return packet_send;
}

information_packet_t createPacket(unsigned char command, unsigned char option, unsigned char type, abstract_packet_t * packet) {
    information_packet_t information;
    information.command = command;
    information.option = option;
    information.type = type;
    if (option == DATA) {
        switch (type) {
            case HASHMAP_DEFAULT:
                information.length = LNG_HEAD_INFORMATION_PACKET + hashmap_default[command];
                break;
            case HASHMAP_MOTION:
                information.length = LNG_HEAD_INFORMATION_PACKET + hashmap_motion[command];
                break;
            default:
                //TODO throw
                break;
        }
    } else {
        information.length = LNG_HEAD_INFORMATION_PACKET;
    }
    if (packet != NULL) {
        memcpy(&information.packet, packet, sizeof (abstract_packet_t));
    }
    return information;
}

information_packet_t createDataPacket(unsigned char command, unsigned char type, abstract_packet_t * packet) {
    return createPacket(command, DATA, type, packet);
}