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

//#include <stdint.h>        /* Includes uint16_t definition   */
//#include <stdbool.h>       /* Includes true/false definition */
//#include <string.h>

#include <system/events.h>

#include "communication/serial.h"

#include "system/system.h"

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

#define SERIAL "SERIAL"
string_data_t _MODULE_SERIAL = {SERIAL, sizeof(SERIAL)};

//UART
#define BAUDRATE 115200
//#define BAUDRATE 57600
#define BRGVAL   ((FCY/BAUDRATE)/16)-1

/*! Array for DMA UART buffer */
unsigned char BufferTx[MAX_BUFF_TX] __attribute__((space(dma)));
hEvent_t parseEvent = INVALID_EVENT_HANDLE;

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
/**
 * In a packet we have more messages. A typical data packet
 * have this structure:
 * -------------------------- ---------------------------- -----------------------
 * | Length | CMD | DATA ... | Length | CMD | INFORMATION |Length | CMD | ... ... |
 * -------------------------- ---------------------------- -----------------------
 *    1        2 -> length    length+1 length+2 length+3   ...
 * It is possible to have different type of messages:
 * * Message with data (D)
 * * Message with state information:
 *      * (R) request data
 *      * (A) ack
 *      * (N) nack
 * We have three parts to elaborate and send a new packet (if required)
 * 1. [SAVING] The first part of this function split packets in a
 * list of messages to compute.
 * 2. [COMPUTE] If message have a data in tail, start compute and return a
 * new ACK or NACK message to append in a new packet. If is a request
 * message (R), the new message have in tail the data required.
 * 3. [SEND] Encoding de messages and transform in a packet to send.
 * *This function is a long function*
 * @return time to compute parsing packet
 */
void parse_packet(int argc, int* argv) {
    packet_information_t list_data[BUFFER_LIST_PARSING];
    size_t len = 0;

    if(parser(&list_data[0], &len) && len != 0) {
        //Build a new message
        packet_t send = encoder(&list_data[0], len);
        // Send a new packet
        serial_send(send);
    }
}

void SerialComm_Init(void) {
    InitUART1();
    InitDMA1();
    
    init_hashmap_packet();          ///< Initialize hash map packet
    init_buff_serial_error();       ///< Initialize buffer serial error
    /// Register module
    hModule_t serial_module = register_module(&_MODULE_SERIAL);
    /// Register event
    parseEvent = register_event_p(serial_module, &parse_packet, EVENT_PRIORITY_LOW);
}

void serial_send(packet_t packet) {
    
    //Wait to complete send packet from UART1 and DMA1.
    while ((U1STAbits.TRMT == 0) && (DMA1CONbits.CHEN == 0));
    //Build a message to send to serial
    build_pkg(BufferTx, packet);
    
    DMA1CNT = (HEAD_PKG + packet.length + 1) - 1; // # of DMA requests
    DMA1CONbits.CHEN = 1; // Enable DMA1 Channel
    DMA1REQbits.FORCE = 1; // Manual mode: Kick-start the 1st transfer
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

void __attribute__((interrupt, auto_psv)) _DMA1Interrupt(void) {
    IFS0bits.DMA1IF = 0; // Clear the DMA1 Interrupt Flag
}
