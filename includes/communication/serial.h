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

    #include <serial/or_message.h>
    #include <serial/or_frame.h>
    
    //Dimension of list messages to decode in a packet
    #define BUFFER_LIST_PARSING 10

    /** 
     * Initialization UART1 for communication and
     * Initialization DMA1 for UART Tx transmition
     */
    void SerialComm_Init(void);

    /**
     * Send serial message to uart
     * @param packet
     */
    void serial_send(packet_t packet);

#ifdef	__cplusplus
}
#endif

#endif	/* SERIAL_H */

