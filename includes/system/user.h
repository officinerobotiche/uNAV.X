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

#ifndef USER_H
#define	USER_H

#ifdef	__cplusplus
extern "C" {
#endif

    #include <stddef.h>

/******************************************************************************/
/* User Level #define Macros                                                  */
/******************************************************************************/

    #define LED1 _LATC6              // Led 1 green
    #define LED2 _LATC7              // Led 2 green
    #define LED3 _LATC8              // Led 3 yellow
    #define LED4 _LATC9              // Led 4 red

    #define MOTOR_ENABLE1 _LATA7     // Enable Motore 1
    #define MOTOR_ENABLE2 _LATA10    // Enable Motore 2
    #define PID_FLAG IFS0bits.OC1IF
    #define PARSER_FLAG IFS0bits.OC2IF
    #define DEAD_RECKONING_FLAG IFS3bits.RTCIF

    #define SGN(x)  ( ((x) < 0) ?  -1 : ( ((x) == 0 ) ? 0 : 1) )

/******************************************************************************/
/* User Function Prototypes                                                   */
/******************************************************************************/

    /* TODO User level functions prototypes (i.e. InitApp) go here */

    /**
     * I/O and Peripheral Initialization
     */
    void InitApp(void);
    /**
     * protected memcpy
     * @param reg
     * @param destination
     * @param source
     * @param num
     */
    void protectedMemcpy(unsigned reg, void *destination, const void *source, size_t num);

    /**
     * Evaluate max value of array on float
     * @param myArray
     * @param size
     * @return
     */
    int maxValue(float myArray[], size_t size);

    void blinkflush();

#ifdef	__cplusplus
}
#endif

#endif	/* USER_H */