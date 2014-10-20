/*
 * File:   packet.h
 * Author: raffaello
 *
 * Created on 12 giugno 2013, 15.29
 */

#ifndef PACKET_H
#define	PACKET_H

/** Serial **/

#include <stdint.h>

#define MOTION_CONTROL
//#define NAVIGATION_BOARD

/*******/

#ifdef MOTION_CONTROL
#include "packet/motion.h"
#endif
#ifdef NAVIGATION_BOARD
#include "navigation.h"
#endif

// Define dimension buffer for packet and type of packet

#define MAX_TX_BUFF 200
#define MAX_RX_BUFF 200
#define BUFF_SERIAL_ERROR 13
#define BUFF_ALL_PROCESS 10
#define BUFF_NAME_PROCESS 20

#define REQUEST 'R'
#define DATA 'D'
#define ACK 'K'
#define NACK 'N'
#define LNG_HEAD_INFORMATION_PACKET 4

//

//#define LNG_ENABLE 2

typedef struct parameter_system {
    int16_t step_timer;
    int16_t int_tm_mill;
} parameter_system_t;
#define LNG_PARAMETER_SYSTEM sizeof(parameter_system_t)

typedef struct error_pkg {
    int16_t number[BUFF_SERIAL_ERROR];
} error_pkg_t;
#define LNG_ERROR_PKG sizeof(error_pkg_t)

#define SERVICE_BUFF 20
typedef struct services {
    char command;
    unsigned char buffer[SERVICE_BUFF];
} services_t;
#define LNG_SERVICES sizeof(services_t)

typedef struct process_buffer {
    int16_t name;
    char buffer[BUFF_NAME_PROCESS];
} process_buffer_t;
#define LNG_NAME_PROCESS sizeof(process_buffer_t)

typedef struct process {
    int16_t length;
    int16_t idle;
    int16_t parse_packet;
    int16_t process[BUFF_ALL_PROCESS];
} process_t;
#define LNG_PROCESS sizeof(process_t)

typedef struct packet_data {
    unsigned int length;
    unsigned char buffer[MAX_RX_BUFF];
    unsigned int time;
} packet_t;

typedef union abstract_packet {
    //unsigned char buffer[MAX_RX_BUFF];
    process_t process;
    services_t services;
    error_pkg_t error_pkg;
    parameter_system_t parameter_system;
    process_buffer_t process_name;
#ifdef MOTION_CONTROL
    ABSTRACT_PACKET_MOTION
#endif
#ifdef NAVIGATION_BOARD
    ABSTRACT_PACKET_NAVIGATION
#endif
} abstract_packet_t;

typedef struct information_packet {
    unsigned char length;
    unsigned char option;
    unsigned char type;
    unsigned char command;
    abstract_packet_t packet;
} information_packet_t;

typedef union buffer_packet {
    information_packet_t information_packet;
    unsigned char buffer[MAX_RX_BUFF];
} buffer_packet_u;

#define SERVICES 0
#define TIME_PROCESS 1
#define PRIORITY_PROCESS 2
#define FRQ_PROCESS 3
#define PARAMETER_SYSTEM 4
#define ERROR_SERIAL 5
#define NAME_PROCESS 6

//For Services
#define RESET '*'
#define DATE_CODE 'd'
#define NAME_BOARD 'n'
#define VERSION_CODE 'v'
#define AUTHOR_CODE 'a'

#define HASHMAP_DEFAULT 'D'
//static unsigned int hashmap_default[10];

#define INITIALIZE_HASHMAP_DEFAULT hashmap_default[SERVICES] = LNG_SERVICES;    \
                                   hashmap_default[TIME_PROCESS] = LNG_PROCESS; \
                                   hashmap_default[PRIORITY_PROCESS] = LNG_PROCESS; \
                                   hashmap_default[FRQ_PROCESS] = LNG_PROCESS; \
                                   hashmap_default[PARAMETER_SYSTEM] = LNG_PARAMETER_SYSTEM; \
                                   hashmap_default[ERROR_SERIAL] = LNG_ERROR_PKG;       \
                                   hashmap_default[NAME_PROCESS] = LNG_NAME_PROCESS;


#endif	/* PACKET_H */

