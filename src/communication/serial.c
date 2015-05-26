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

#include <serial/or_message.h>
#include <serial/or_frame.h>
#include "communication/serial.h"

#include "system/user.h"
#include "system/system.h"

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

/*! Array for DMA UART buffer */
unsigned char BufferTx[MAX_BUFF_TX] __attribute__((space(dma)));

/** GLOBAL VARIBLES */
// From system/system.c
extern system_parameter_t parameter_system;
// From communication/serial.c
extern system_error_serial_t serial_error;
extern packet_t receive_pkg;
extern char receive_header;

/******************************************************************************/
/* Communication Functions                                                    */
/******************************************************************************/

void serial_send(char header, packet_t packet) {
    
    //Wait to complete send packet from UART1 and DMA1.
    while ((U1STAbits.TRMT == 0) && (DMA1CONbits.CHEN == 0));
    //Build a message to send to serial
    build_pkg(BufferTx, header, packet);
    
    DMA1CNT = (HEAD_PKG + packet.length + 1) - 1; // # of DMA requests
    DMA1CONbits.CHEN = 1; // Enable DMA1 Channel
    DMA1REQbits.FORCE = 1; // Manual mode: Kick-start the 1st transfer
}

int parse_packet() {
    unsigned int t = TMR1; // Timing function
    packet_information_t list_data[BUFFER_LIST_PARSING];
    unsigned short len = 0;

    if(parser(&list_data[0], &len) && len != 0) {
        //Build a new message
        packet_t send = encoder(&list_data[0], len);
        // Send a new packet
        serial_send(receive_header, send);
    }
    return TMR1 - t; // Time of execution
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