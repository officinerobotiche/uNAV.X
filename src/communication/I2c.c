/*
 * Copyright (C) 2014 Officine Robotiche
 * Authors: Guido Ottaviani, Raffaello Bonghi
 * email:  guido@guiott.com, raffaello.bonghi@officinerobotiche.it
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
 * 
 * Original code:
 * https://code.google.com/p/gentlenav/source/browse/trunk/libUDB/I2C1.c
 */

/* ////////////////////////////////////////////////////////////////////////////
 ** It contains all functions related to I2C communication
 ** It works fully interrupt driven using a queued method to execute
 ** multiple communication sequences with multiple peripherals
 **

 * Start a transaction and take ownership of I2C bus.
 * returns false if I2C is busy or not initialized
 * command = command specific to device
 * ptxData = pointer to transmit data buffer
 * prxData = pointer to receive data buffer
 * txSize = size of transmited data in bytes
 * rxSize = size of received data in bytes
 * pCallback = pointer to callback function for finish or error.
 *
/////////////////////////////////////////////////////////////////////////////*/

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

#include <stdbool.h>       /* Includes true/false definition */

#include <system/events.h>
#include <peripherals/i2c.h>

#include "communication/I2c.h"

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

hardware_bit_t MI2C1IF = REGISTER_INIT(IFS1, 1);

/******************************************************************************/
/* Parsing functions                                                          */
/******************************************************************************/

void Init_I2C(void) {
    I2C_Init(&MI2C1IF, &I2CCON, &I2CSTAT, &I2CTRN, &I2CRCV);   ///< Open I2C module
}

void __attribute__((__interrupt__, __no_auto_psv__)) _MI2C1Interrupt(void) {
    
    I2C_manager();
    _MI2C1IF = 0; // clear the interrupt
}
