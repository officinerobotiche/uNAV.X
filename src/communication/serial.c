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

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

/*! Pointer to function, initialized for pkg_header */
int (*pkg_parse) (unsigned char inchar) = &pkg_header;
/*! Array for DMA UART buffer */
unsigned char BufferTx[MAX_TX_BUFF] __attribute__((space(dma)));
/*! Receive packet */
packet_t receive_pkg;
char receive_header;
unsigned int index_data = 0;
error_pkg_t serial_error;

/******************************************************************************/
/* Comunication Functions                                                     */
/******************************************************************************/

/**
 * Init buffer serial_error to zero
 */
void init_buff_serial_error(){
    memset(serial_error.number, 0, BUFF_SERIAL_ERROR);
}

/**
 * Function called on _U1RXInterrupt for decode packet
 * Data structure:
 * ------------------------------------------------
 * | HEADER | LENGTH |       DATA           | CKS |
 * ------------------------------------------------
 *     1        2             3 -> n          n+1
 *
 * Only element of packet have a relative function to decode
 * 1) Header -> pkg_header
 * 2) Length -> pkg_length
 * 3 to n+1) Data -> pkg_data
 * @param rxchar char character received from interrupt
 * @return boolean result from pointer function called on decode
 */
int decode_pkgs(unsigned char rxchar) {
    return (*pkg_parse)(rxchar);
}

/**
 * First function to decode Header from Serial interrupt
 * Verify if rxchar is a HEADER_SYNC or HEADER_ASYNC then
 * update pointer function pkg_parse for next function pkg_length
 * and save type of header, else save error header and going to pkg_error
 * @param rxchar character received from interrupt
 * @return boolean result, only false
 */
int pkg_header(unsigned char rxchar) {
    if ((rxchar == HEADER_SYNC) || (rxchar == HEADER_ASYNC)) {
        receive_header = rxchar;    // Save
        pkg_parse = &pkg_length;
        return false;
    } else {
        return pkg_error(ERROR_HEADER);
    }
}

/**
 * Second function for decode packet, this function is to able to verify
 * length of packet. If length (rxchar) is larger than MAX_RX_BUFF
 * call function pkg_error with ERROR_LENGTH. Else change function to call
 * pkg_data and save information on length in receive_pkg
 * @param rxchar character received from interrupt
 * @return boolean result, only false
 */
int pkg_length(unsigned char rxchar) {
    if (rxchar > MAX_RX_BUFF) {
        return pkg_error(ERROR_LENGTH);
    } else {
        pkg_parse = &pkg_data;
        receive_pkg.length = rxchar;
        return false;
    }
}

/**
 * Function for decode packet, save in receive_pkg.buffer all bytes. In (n+1)
 * start pkg_checksum() function to verify correct receive packet.
 * @param rxchar character received from interrupt
 * @return boolean result. True if don't have any error else start pkg_error
 * and return false.
 */
int pkg_data(unsigned char rxchar) {
    int cks_clc;
    if ((index_data + 1) == (receive_pkg.length + 1)) {
        pkg_parse = &pkg_header; //Restart parse serial packet
        if ((cks_clc = pkg_checksum(receive_pkg.buffer, 0, index_data)) == rxchar) { //checksum data
            index_data = 0; //flush index array data buffer
            return true;
        } else {
            bool t = pkg_error(ERROR_CKS);
            return t;
        }
    } else {
        receive_pkg.buffer[index_data] = rxchar;
        index_data++;
        return false;
    }
}

/**
 * Reset all function about decode packet and save increase counter error
 * for type.
 * @param error Number of type error.
 * @return same number error.
 */
int pkg_error(int error) {
    index_data = 0;
    pkg_parse = &pkg_header; //Restart parse serial packet
    serial_error.number[(-error - 1)] += 1;
    return error;
}

/**
 * Function to evaluate checksum. Count all bytes in a Buffer and return
 * number for checksum.
 * @param Buffer It's a buffer to sum all bytes
 * @param FirstIndx The number for first element buffer to count all bytes.
 * @param LastIndx The number for last element buffer.
 * @return number evaluated for sum bytes
 */
unsigned char pkg_checksum(volatile unsigned char* Buffer, int FirstIndx, int LastIndx) {
    unsigned char ChkSum = 0;
    int ChkCnt;
    for (ChkCnt = FirstIndx; ChkCnt < LastIndx; ChkCnt++) {
        ChkSum += Buffer[ChkCnt];
    }
    return ChkSum;
}

/**
 * Function to send a packet. Copy on DMA buffer all bytes 
 * @param header type of packet. SYNC or ASYNC packet
 * @param packet packet to send.
 */
void pkg_send(char header, packet_t packet) {

    //Wait to complete send packet from UART1 and DMA1.
    while ((U1STAbits.TRMT == 0) && (DMA1CONbits.CHEN == 0));

    int i;
    BufferTx[0] = header;
    BufferTx[1] = packet.length;

    //Copy all element to DMA buffer
    for (i = 0; i < packet.length; i++) {
        BufferTx[i + HEAD_PKG] = packet.buffer[i];
    }

    // Create a checksum
    BufferTx[packet.length + HEAD_PKG] = pkg_checksum(BufferTx, HEAD_PKG, packet.length + HEAD_PKG);

    DMA1CNT = (HEAD_PKG + packet.length + 1) - 1; // # of DMA requests
    DMA1CONbits.CHEN = 1; // Enable DMA1 Channel
    DMA1REQbits.FORCE = 1; // Manual mode: Kick-start the 1st transfer
}
