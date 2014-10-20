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

#ifndef SERIAL_H
#define	SERIAL_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "packet/packet.h"

#define HEADER_SYNC '#'
#define HEADER_ASYNC '@'
#define HEAD_PKG 2

#define ERROR_FRAMMING -1   //Framing Error bit
#define ERROR_OVERRUN -2    //overrun error

#define ERROR_HEADER -3
#define ERROR_LENGTH -4
#define ERROR_DATA -5
#define ERROR_CKS -6
#define ERROR_CMD -7
#define ERROR_NACK -8
#define ERROR_OPTION -9
#define ERROR_PKG -10
#define ERROR_CREATE_PKG -11

    /******************************************************************************/
    /* System Function Prototypes                                                 */
    /******************************************************************************/

    /**
     * Init buffer serial_error to zero
     */
    void init_buff_serial_error();

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
    void pkg_send(char header, packet_t packet);

    /**
     * First function to decode Header from Serial interrupt
     * Verify if rxchar is a HEADER_SYNC or HEADER_ASYNC then
     * update pointer function pkg_parse for next function pkg_length
     * and save type of header, else save error header and going to pkg_error
     * @param rxchar character received from interrupt
     * @return boolean result, only false
     */
    int pkg_header(unsigned char rxchar);
    
    /**
     * Second function for decode packet, this function is to able to verify
     * length of packet. If length (rxchar) is larger than MAX_RX_BUFF
     * call function pkg_error with ERROR_LENGTH. Else change function to call
     * pkg_data and save information on length in receive_pkg
     * @param rxchar character received from interrupt
     * @return boolean result, only false
     */
    int pkg_length(unsigned char rxchar);

    /**
     * Function for decode packet, save in receive_pkg.buffer all bytes. In (n+1)
     * start pkg_checksum() function to verify correct receive packet.
     * @param rxchar character received from interrupt
     * @return boolean result. True if don't have any error else start pkg_error
     * and return false.
     */
    int pkg_data(unsigned char rxchar);
    
    /**
     * Reset all function about decode packet and save increase counter error
     * for type.
     * @param error Number of type error.
     * @return same number error.
     */
    int pkg_error(int error);

    /**
     * Function to evaluate checksum. Count all bytes in a Buffer and return
     * number for checksum.
     * @param Buffer It's a buffer to sum all bytes
     * @param FirstIndx The number for first element buffer to count all bytes.
     * @param LastIndx The number for last element buffer.
     * @return number evaluated for sum bytes
     */
    unsigned char pkg_checksum(volatile unsigned char* Buffer, int FirstIndx, int LastIndx);

    /**
     * Function to send a packet. Copy on DMA buffer all bytes 
     * @param header type of packet. SYNC or ASYNC packet
     * @param packet packet to send.
     */
    int decode_pkgs(unsigned char rxchar);

#ifdef	__cplusplus
}
#endif

#endif	/* SERIAL_H */

