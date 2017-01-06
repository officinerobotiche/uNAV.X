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

/******************************************************************************/
/*	INCLUDE																	  */
/******************************************************************************/
    
    #include <or_bus/frame.h>
    #include <or_peripherals/UART/UART.h>
    
    //Dimension of list messages to decode in a packet
    #define BUFFER_LIST_PARSING 10

/******************************************************************************/
/*	FUNCTIONS							 									  */
/******************************************************************************/
    
    /**
     * Initialization UART1 for communication and Initialization DMA1 for UART transmission
     * @param uart The UART controller
     * @param frame The OR BUS Frame controller
     */
    void UART1_Init(UART_t *uart, OR_BUS_FRAME_t *frame);
    /**
     * @brief UART1 callback controller
     * @param rxdata data received
     */
    void UART1_read_callback(unsigned char rxdata);
    /**
     * @brief UART writer with DMA
     * @param uart The UART controller
     * @param buff The buffer to send
     * @param size The size of buffer
     */
    void UART1_DMA_write(void *uart, unsigned char* buff, size_t size);

    

#ifdef	__cplusplus
}
#endif

#endif	/* SERIAL_H */

