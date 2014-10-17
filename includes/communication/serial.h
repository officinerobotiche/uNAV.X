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

    //init serial buffer
    void init_buff_serial_error();
    //Function to encoding packet
    void pkg_send(char header, packet_t packet);
    //Function for decoding packet
    int decode_pkgs(unsigned char rxchar);
    int pkg_header(unsigned char rxchar);
    int pkg_length(unsigned char rxchar);
    int pkg_data(unsigned char rxchar);
    //For encoding and decoding
    unsigned char pkg_checksum(volatile unsigned char* Buffer, int FirstIndx, int LastIndx);
    int pkg_error(int error);


#ifdef	__cplusplus
}
#endif

#endif	/* SERIAL_H */

