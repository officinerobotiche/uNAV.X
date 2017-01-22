/*
 * Copyright (C) 2014-2017 Officine Robotiche
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

/* Standard include file. */
#include <stdlib.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
    
/******************************************************************************/
/* System Level #define Macros                                                */
/******************************************************************************/
    
typedef void * xComPortHandle;
    
/******************************************************************************/
/* System Function Prototypes                                                 */
/******************************************************************************/
/**
 * Physical initialization Serial port
 * @param ulWantedBaud The required baud rate
 * @param uxQueueLength The length of the queue
 * @return the status of serial port initialization
 */
xComPortHandle xSerialPortInitMinimal( unsigned long ulWantedBaud, unsigned portBASE_TYPE uxQueueLength );

signed portBASE_TYPE xSerialPutBuff(xComPortHandle pxPort, signed char* buff, size_t size);
/**
 * Get char from serial port
 * @param pxPort The number of serial port
 * @param pcRxedChar The data to save
 * @param xBlockTime The time to wait
 * @return The status of function
 */
signed portBASE_TYPE xSerialGetChar( xComPortHandle pxPort, signed char *pcRxedChar, TickType_t xBlockTime );

#ifdef	__cplusplus
}
#endif

#endif	/* SERIAL_H */

