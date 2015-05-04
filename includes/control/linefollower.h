/*
 * Copyright (C) 2015 Officine Robotiche
 * Author: Mauro Soligo
 * email:  mauro.soligo@officinerobotiche.it
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


#ifndef LINEFOLLOWER_H
#define	LINEFOLLOWER_H

    /**************************************************************************/
    /* System Level #define Macros                                            */
    /**************************************************************************/
    #define INPUT       1
    #define OUTPUT      0

    #define PORT_IO0    _TRISA9
    #define PORT_IO1    _TRISC0
    #define PORT_IO2    _TRISC1
    #define PORT_IO3    _TRISC2
    #define PORT_IO4    _TRISC3
    #define PORT_IO5    _TRISA4
    #define PORT_IO6    _TRISB4

    #define PIN_IO0     PORTAbits.RA9
    #define PIN_IO1     PORTCbits.RC0       
    #define PIN_IO2     PORTCbits.RC1
    #define PIN_IO3     PORTCbits.RC2
    #define PIN_IO4     PORTCbits.RC3
    #define PIN_IO5     PORTAbits.RA4
    #define PIN_IO6     PORTBbits.RB4
    

    /**************************************************************************/
    /* LineFollower extern variables                                          */
    /**************************************************************************/

    
    /**************************************************************************/
    /* System Function Prototypes                                             */
    /**************************************************************************/

    /**
     * main function of line following algoritms.
     */
    void linefollowing(void);

    /*
     * IR senso reading related functions
     */
    void IRsensor_CapacitorDisCharge(void);    
    void IRsensor_StartMeasure(void);


#endif	/* LINEFOLLOWER_H */
