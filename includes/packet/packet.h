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

#define PACKETS_MOTION
#define PACKETS_MOTOR

/*******/

#ifdef PACKETS_MOTION
#include "packet/motion.h"
#endif
#ifdef PACKETS_MOTOR
#include "packet/motor.h"
#endif
#ifdef PACKETS_NAVIGATION
#include "packet/navigaation.h"
#endif

/** Buffer dimensions */
// Dimension for UART transmit buffer
#define MAX_BUFF_TX 200
// Dimension for UART receive buffer
#define MAX_BUFF_RX 200
// Type of serial errors
#define MAX_BUFF_ERROR_SERIAL 13
// Numbers of process names 
#define MAX_BUFF_TASK_NAME 20
// Dimension services buffer
#define MAX_BUFF_SERVICE 20

/** Type of option messages */
// Request data
#define PACKET_REQUEST  'R'
// Messages with data
#define PACKET_DATA     'D'
// ACK
#define PACKET_ACK      'K'
// NACK
#define PACKET_NACK     'N'
// Length of information packet (without data)
#define LNG_HEAD_INFORMATION_PACKET 4

/**
 * Define messages about parameter system:
     * * Clock of System timer
     * * Clock in milliseconds
 */
typedef struct _system_parameter {
    int16_t step_timer;
    int16_t int_tm_mill;
} system_parameter_t;
#define LNG_SYSTEM_PARAMETER sizeof(system_parameter_t)

/**
 * Service messages about number of error on serial communication
 * see on communication/serial.h type of errors
 */
typedef struct _system_error_serial {
    int16_t number[MAX_BUFF_ERROR_SERIAL];
} system_error_serial_t;
#define LNG_SYSTEM_ERROR_SERIAL sizeof(system_error_serial_t)

/**
 * Services messages for control board:
 * * reset board
 * * date code
 * * name board
 * * version code
 * * author code
 */
typedef struct _system_service {
    char command; // TODO insert a list of commands or a reference to the list of commands
    unsigned char buffer[MAX_BUFF_SERVICE];
} system_service_t;
#define LNG_SYSTEM_SERVICE sizeof(system_service_t)

/**
 * Information about task running on firmware
 * * HASHMAP type of task
 * * number of task
 * * data required or read
 */
typedef struct _system_task {
    uint8_t hashmap;
    uint8_t number;
    uint8_t data;
} system_task_t;
#define LNG_SYSTEM_TASK sizeof(system_task_t)

/**
 * Information about processes running on board. We have standards process:
 * * time in idle
 * * time for parsing packet
 * * list for others processes
 */
typedef struct _system_task_name {
    uint8_t hashmap;
    uint8_t number;
    char data[MAX_BUFF_TASK_NAME];
} system_task_name_t;
#define LNG_SYSTEM_TASK_NAME sizeof(system_task_name_t)

/**** EO Messages ****/

/**
 * This is a definition to convert packets in a big data packet to send in 
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
 * Union for conversion all type of packets in a standard packets
 */
typedef union _message_abstract {
    system_task_name_t system_task_name;
    system_task_t system_task;
    system_service_t system_service;
    system_error_serial_t system_error_serial;
    system_parameter_t system_parameter;
#ifdef PACKETS_MOTION
    ABSTRACT_MESSAGE_MOTOR
#endif
#ifdef PACKETS_MOTOR
    ABSTRACT_MESSAGE_MOTION
#endif
#ifdef PACKETS_NAVIGATION
    ABSTRACT_MESSAGE_NAVIGATION
#endif
} message_abstract_u;

/**
 * Structure with information about a packet:
 * * length for packet
 * * information about packet (in top on this file):
 *      * (R) Request data
 *      * (D) Packet with data
 *      * (K) ACK
 *      * (N) NACK
 * * type packet:
 *      * (D) Default messages (in top on this file)
 *      * other type messages (in UNAV file)
 * * command message
 */
typedef struct _packet_information {
    unsigned char length;
    unsigned char option;
    unsigned char type;
    unsigned char command;
    message_abstract_u message;
} packet_information_t;

/**
 * Union to quickly transform information_packet_t in a buffer to add in
 * packet_t
 */
typedef union _packet_buffer {
    packet_information_t packet_information;
    unsigned char buffer[MAX_BUFF_RX];
} packet_buffer_u;

/**
 * Structure with information about packet to send with serial port:
 * * length of packet
 * * buffer with data
 * * time to send packet (NOT IN USE)
 */
typedef struct _packet {
    unsigned int length;
    unsigned char buffer[MAX_BUFF_RX];
    unsigned int time;
} packet_t;

//Number association for standard messages
#define SYSTEM_SERVICE          0
#define SYSTEM_TASK_NAME        1
#define SYSTEM_TASK_TIME        2
#define SYSTEM_TASK_PRIORITY    3
#define SYSTEM_TASK_FRQ         4
#define SYSTEM_TASK_NUM         5
#define SYSTEM_PARAMETER        6
#define SYSTEM_SERIAL_ERROR     7

//Names for type services
#define SERVICE_RESET           '*'
#define SERVICE_CODE_DATE       'd'
#define SERVICE_CODE_VERSION    'v'
#define SERVICE_CODE_AUTHOR     'a'
#define SERVICE_CODE_BOARD_TYPE 't'
#define SERVICE_CODE_BOARD_NAME 'n'

//Name for HASHMAP with information about standard messages
#define HASHMAP_SYSTEM          'S'
#define HASHMAP_SYSTEM_NUMBER   10

// Definition on communication/parsing_packet.c
//static unsigned int hashmap_system[HASHMAP_SYSTEM_NUMBER];

/**
 * Table with conversion number message in a length for data messages
 */
#define HASHMAP_SYSTEM_INITIALIZE hashmap_system[SYSTEM_SERVICE] = LNG_SYSTEM_SERVICE;           \
                                  hashmap_system[SYSTEM_TASK_NAME] = LNG_SYSTEM_TASK_NAME;       \
                                  hashmap_system[SYSTEM_TASK_TIME] = LNG_SYSTEM_TASK;            \
                                  hashmap_system[SYSTEM_TASK_PRIORITY] = LNG_SYSTEM_TASK;        \
                                  hashmap_system[SYSTEM_TASK_FRQ] = LNG_SYSTEM_TASK;             \
                                  hashmap_system[SYSTEM_TASK_NUM] = LNG_SYSTEM_TASK;             \
                                  hashmap_system[SYSTEM_PARAMETER] = LNG_SYSTEM_PARAMETER;       \
                                  hashmap_system[SYSTEM_SERIAL_ERROR] = LNG_SYSTEM_ERROR_SERIAL;

#endif	/* PACKET_H */

