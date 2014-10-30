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

#ifndef PACKET_H
#define	PACKET_H

#include <stdint.h>

/** Serial packets
 * On this header file, we have all definition about communication and defined
 * the standard packets, to send information :
 * * Parameter system
 * * error packages receives from serial ports
 * * Services messages
 *      * reset board
 *      * date code
 *      * name board
 *      * version code
 *      * author code
 * * Messages with all names processes
 * * Information about processes
 *
 * Other messages for this board are in packet/unav.h
 */

#define UNAV

/*******/

#ifdef UNAV
#include "packet/unav.h"
#endif

/** Buffers dimension */
// Dimension for UART transmit buffer
#define MAX_TX_BUFF 200
// Dimension for UART receive buffer
#define MAX_RX_BUFF 200
// Type of serial errors
#define BUFF_SERIAL_ERROR 13
// Numbers of process
#define BUFF_ALL_PROCESS 10
// Numbers of names process
#define BUFF_NAME_PROCESS 20
// Dimension services buffer
#define SERVICE_BUFF 20

/** Type of option messages */
// Requesta data
#define REQUEST 'R'
// Messages with data
#define DATA 'D'
// ACK
#define ACK 'K'
// NACK
#define NACK 'N'
// Length of information packet (without data)
#define LNG_HEAD_INFORMATION_PACKET 4

/**
 * Define messages about parameter system:
 * * Clock of System timer
 * * Colck in milliseconds
 */
typedef struct parameter_system {
    int16_t step_timer;
    int16_t int_tm_mill;
} parameter_system_t;
#define LNG_PARAMETER_SYSTEM sizeof(parameter_system_t)

/**
 * Service messages about number of error on serial communication
 * see on communication/serial.h type of errors
 */
typedef struct error_pkg {
    int16_t number[BUFF_SERIAL_ERROR];
} error_pkg_t;
#define LNG_ERROR_PKG sizeof(error_pkg_t)

/**
 * Services messages for control board:
 * * reset board
 * * date code
 * * name board
 * * version code
 * * author code
 */
typedef struct services {
    char command;
    unsigned char buffer[SERVICE_BUFF];
} services_t;
#define LNG_SERVICES sizeof(services_t)

/**
 * Services messages for all names processes
 */
typedef struct process_buffer {
    int16_t name;
    char buffer[BUFF_NAME_PROCESS];
} process_buffer_t;
#define LNG_NAME_PROCESS sizeof(process_buffer_t)

/**
 * Information about processes on board. We have standards process:
 * * time in idle
 * * time for parsing packet
 * * list for others processes
 */
typedef struct process {
    int16_t length;
    int16_t idle;
    int16_t parse_packet;
    int16_t process[BUFF_ALL_PROCESS];
} process_t;
#define LNG_PROCESS sizeof(process_t)
/* EO Messages */

/**
 * This is a definition for convertion packets in a big data packet to send in 
 * a serial communication. 
 * For all packet we have this transformation:
 * 1. UNION abstract_packet_u
 * 2. STRUCT information_packet_t
 * 3. UNION buffer_packet_u
 *
 * Finally a function convert all information_packet_t in a
 * long data packet with name:
 * 4. STRUCT packet_t
 */

/**
 * Struct with information about packet to send with serial port:
 * * length of packet
 * * buffer with data
 * * time to send packet (NOT IN USE)
 */
typedef struct packet_data {
    unsigned int length;
    unsigned char buffer[MAX_RX_BUFF];
    unsigned int time;
} packet_t;

/**
 * Union for convertion all type of packets in a standard packets
 */
typedef union abstract_message {
    //unsigned char buffer[MAX_RX_BUFF];
    process_t process;
    services_t services;
    error_pkg_t error_pkg;
    parameter_system_t parameter_system;
    process_buffer_t process_name;
#ifdef UNAV
    ABSTRACT_MESSAGE_UNAV
#endif
#ifdef NAVIGATION_BOARD
    ABSTRACT_PACKET_NAVIGATION
#endif
} abstract_message_t;

/**
 * Struct with information about a packet:
 * * length for packet
 * * information about packet (in top on this file):
 *      * (R) Request data
 *      * (D) Packet with data
 *      * (A) ACK
 *      * (N) NACK
 * * type packet:
 *      * (D) Default messages (in top on this file)
 *      * other type messages (in UNAV file)
 * * command message
 */
typedef struct information_packet {
    unsigned char length;
    unsigned char option;
    unsigned char type;
    unsigned char command;
    abstract_message_t packet;
} information_packet_t;

/**
 * Union for quickly transform information_packet_t in a buffer to add in
 * packet_t
 */
typedef union buffer_packet {
    information_packet_t information_packet;
    unsigned char buffer[MAX_RX_BUFF];
} buffer_packet_u;

//Number association for standard messages
#define SERVICES 0
#define TIME_PROCESS 1
#define PRIORITY_PROCESS 2
#define FRQ_PROCESS 3
#define PARAMETER_SYSTEM 4
#define ERROR_SERIAL 5
#define NAME_PROCESS 6

//Names for type service's messages
#define RESET '*'
#define DATE_CODE 'd'
#define NAME_BOARD 'n'
#define VERSION_CODE 'v'
#define AUTHOR_CODE 'a'

//Name for HASHMAP with information about standard messages
#define HASHMAP_DEFAULT 'D'
#define HASHMAP_DEFAULT_NUMBER 10

// Definition on communication/parsing_packet.c
//static unsigned int hashmap_default[HASHMAP_DEFAULT_NUMBER];

/**
 * Table with convertion number message in a length for data messages
 */
#define INITIALIZE_HASHMAP_DEFAULT hashmap_default[SERVICES] = LNG_SERVICES;    \
                                   hashmap_default[TIME_PROCESS] = LNG_PROCESS; \
                                   hashmap_default[PRIORITY_PROCESS] = LNG_PROCESS; \
                                   hashmap_default[FRQ_PROCESS] = LNG_PROCESS; \
                                   hashmap_default[PARAMETER_SYSTEM] = LNG_PARAMETER_SYSTEM; \
                                   hashmap_default[ERROR_SERIAL] = LNG_ERROR_PKG;       \
                                   hashmap_default[NAME_PROCESS] = LNG_NAME_PROCESS;


#endif	/* PACKET_H */

