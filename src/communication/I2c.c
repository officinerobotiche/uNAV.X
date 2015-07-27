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
#include <peripherals/i2c_controller.h>
#include "system/system.h"

#include "communication/I2c.h"

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

#define I2CBRGVAL ( (int)(((1/100e3) - 130E-9) * FCY)-2 ) // 392 // 100 Khz
#define I2C_SDA    _LATB9    //  from _RA3, _RA2, mods per Bill P.
#define I2C_SCL    _LATB8

hardware_bit_t MI2C1IF = REGISTER_INIT(IFS1, 1);

/******************************************************************************/
/* Parsing functions                                                          */
/******************************************************************************/

void reset_I2C(bool state) {
    _MI2C1IF = 0; // clear the I2C master interrupt
    _MI2C1IE = 0; // disable the interrupt
    // pull SDA and SCL low
    I2C_SCL = 0;
    I2C_SDA = 0;
    Nop();
    // pull SDA and SCL high
    I2C_SCL = 1;
    I2C_SDA = 1;
}

void Init_I2C(void) {
    
    I2C1BRG = I2CBRGVAL;
    
    /// Open I2C module
    I2C_Init(&MI2C1IF, &I2CCON, &I2CSTAT, &I2CTRN, &I2CRCV, &reset_I2C);
    
    _MI2C1IP = 6; // I2C at priority 5
    _MI2C1IF = 0; // clear the I2C master interrupt
    _MI2C1IE = 1; // enable the interrupt
}

void __attribute__((__interrupt__, __no_auto_psv__)) _MI2C1Interrupt(void) {
    
    I2C_manager();
    _MI2C1IF = 0; // clear the interrupt
}
