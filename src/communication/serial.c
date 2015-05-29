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

#include <system/events.h>

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
hEvent_t parseEvent = INVALID_HANDLE;

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

void InitUART1(void) {
    U1MODEbits.STSEL = 0; // 1-stop bit
    U1MODEbits.PDSEL = 0; // No Parity, 8-data bits
    U1MODEbits.ABAUD = 0; // Auto-Baud Disabled
    U1MODEbits.BRGH = 0; // Low Speed mode

    U1BRG = BRGVAL; // BAUD Rate Setting on System.h

    U1STAbits.UTXISEL0 = 0; // Interrupt after one Tx character is transmitted
    U1STAbits.UTXISEL1 = 0;

    IEC0bits.U1TXIE = 0; // Disable UART Tx interrupt
    U1STAbits.URXISEL = 0; // Interrupt after one RX character is received

    U1MODEbits.UARTEN = 1; // Enable UART
    U1STAbits.UTXEN = 1; // Enable UART Tx

    IEC4bits.U1EIE = 0;
    IPC2bits.U1RXIP = UART_RX_LEVEL; // Set UART Rx Interrupt Priority Level
    IFS0bits.U1RXIF = 0; // Reset RX interrupt flag
    IEC0bits.U1RXIE = 1; // Enable RX interrupt
}

void InitDMA1(void) {
    //DMA1CON = 0x2001;			// One-Shot, Post-Increment, RAM-to-Peripheral

    DMA1CONbits.CHEN = 0;
    DMA1CONbits.SIZE = 1;
    DMA1CONbits.DIR = 1;
    DMA1CONbits.HALF = 0;
    DMA1CONbits.NULLW = 0;
    DMA1CONbits.AMODE = 0;
    DMA1CONbits.MODE = 1;

    DMA1CNT = MAX_BUFF_TX - 1; // 32 DMA requests
    DMA1REQ = 0x000c; // Select UART1 Transmitter

    DMA1STA = __builtin_dmaoffset(BufferTx);
    DMA1PAD = (volatile unsigned int) &U1TXREG;

    IPC3bits.DMA1IP = UART_TX_LEVEL; // Set DMA Interrupt Priority Level
    IFS0bits.DMA1IF = 0; // Clear DMA Interrupt Flag
    IEC0bits.DMA1IE = 1; // Enable DMA interrupt
}

void SerialComm_Init(void) {
    InitUART1();
    InitDMA1();
    /// Register event
    parseEvent = register_event_p(&parse_packet, EVENT_PRIORITY_LOW);
}

void serial_send(char header, packet_t packet) {
    
    //Wait to complete send packet from UART1 and DMA1.
    while ((U1STAbits.TRMT == 0) && (DMA1CONbits.CHEN == 0));
    //Build a message to send to serial
    build_pkg(BufferTx, header, packet);
    
    DMA1CNT = (HEAD_PKG + packet.length + 1) - 1; // # of DMA requests
    DMA1CONbits.CHEN = 1; // Enable DMA1 Channel
    DMA1REQbits.FORCE = 1; // Manual mode: Kick-start the 1st transfer
}

int parse_packet(void) {
    unsigned int t = TMR1; // Timing function
    packet_information_t list_data[BUFFER_LIST_PARSING];
    size_t len = 0;

    if(parser(&list_data[0], &len) && len != 0) {
        //Build a new message
        packet_t send = encoder(&list_data[0], len);
        // Send a new packet
        serial_send(receive_header, send);
    }
    return TMR1 - t; // Time of execution
}

unsigned int ReadUART1(void) {
    if (U1MODEbits.PDSEL == 3)
        return (U1RXREG);
    else
        return (U1RXREG & 0xFF);
}

void __attribute__((interrupt, auto_psv)) _U1RXInterrupt(void) {
    IFS0bits.U1RXIF = 0; // clear RX interrupt flag

    /* get the data */
    if (U1STAbits.URXDA == 1) {
        if (decode_pkgs(ReadUART1())) {
            trigger_event(parseEvent);
        }
    } else {
        /* check for receive errors */
        if (U1STAbits.FERR == 1) {
            pkg_error(ERROR_FRAMMING);
        }
        /* must clear the overrun error to keep uart receiving */
        if (U1STAbits.OERR == 1) {
            U1STAbits.OERR = 0;
            pkg_error(ERROR_OVERRUN);
        }
    }
}

void save_frame_system(packet_information_t* list_send, size_t* len, packet_information_t* info) {
    message_abstract_u send;
    switch (info->command) {
        case SYSTEM_SERVICE:
            send.system.service = services(info->message.system.service);
            list_send[(*len)++] = createDataPacket(info->command, info->type, &send);
            break;
        case SYSTEM_TASK_PRIORITY:
        case SYSTEM_TASK_FRQ:
            set_process(info->command, info->message.system.task);
            list_send[(*len)++] = createPacket(info->command, PACKET_ACK, info->type, NULL);
            break;
        default:
            list_send[(*len)++] = createPacket(info->command, PACKET_NACK, info->type, NULL);
            break;
    }
}

void send_frame_system(packet_information_t* list_send, size_t* len, packet_information_t* info) {
    message_abstract_u send;
    switch (info->command) {
        case SYSTEM_SERVICE:
            send.system.service = services(info->message.system.service);
            list_send[(*len)++] = createDataPacket(info->command, info->type, &send);
            break;
        case SYSTEM_TASK_PRIORITY:
        case SYSTEM_TASK_FRQ:
        case SYSTEM_TASK_TIME:
        case SYSTEM_TASK_NUM:
            send.system.task = get_process(info->command, info->message.system.task);
            list_send[(*len)++] = createDataPacket(info->command, info->type, &send);
            break;
        case SYSTEM_TASK_NAME:
            send.system.task_name = get_process_name(info->message.system.task_name);
            list_send[(*len)++] = createDataPacket(info->command, info->type, &send);
            break;
        case SYSTEM_PARAMETER:
            send.system.parameter = parameter_system;
            list_send[(*len)++] = createDataPacket(info->command, info->type, &send);
            break;
        case SYSTEM_SERIAL_ERROR:
            send.system.error_serial = serial_error;
            list_send[(*len)++] = createDataPacket(info->command, info->type, &send);
            break;
        default:
            list_send[(*len)++] = createPacket(info->command, PACKET_NACK, info->type, NULL);
            break;
    }
}