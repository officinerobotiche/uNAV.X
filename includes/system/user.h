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
/* µNAV pin map                                                               */
/*                                                                            */
/* Input ADC                                                                  */
/* AN0 -> RA0                                                                 */
/* AN1 -> RA1                                                                 */
/* AN2 -> RB0                                                                 */
/* AN3 -> RB1                                                                 */
/*                                                                            */
/* Input encoder (5V tolerant)                                                */
/* E1CHA -> RB10 (RP10)                                                       */
/* E1CHB -> RB11 (RP11)                                                       */
/* E2CHA -> RB6 (RP6)                                                         */
/* E2CHB -> RB5 (RP5)                                                         */
/*                                                                            */
/* input capture)                                                             */
/* IC1 -> RB10 (RP10)                                                         */
/* IC2 -> RB6 (RP6)                                                           */
/*                                                                            */
/* OUT PWM                                                                    */
/* H1A -> RB14                                                                */
/* H1B -> RB15                                                                */
/* H2A -> RB12                                                                */
/* H2B -> RB13                                                                */
/*                                                                            */
/* H Bridge control                                                           */
/* H1EN   -> RA7                                                              */
/* H2EN   -> RA10                                                             */
/* (note: AU1 e AU2 are NC)                                                   */
/*                                                                            */
/* UART (5V tolerant)                                                         */
/* U1RX -> RC5 (RP21)                                                         */
/* U1TX -> RC4 (RP20)                                                         */
/* U2RX -> RB3 (RP3)                                                          */
/* U2TX -> RB2 (RP2)                                                          */
/*                                                                            */
/* I2C                                                                        */
/* SDA -> RB9                                                                 */
/* SCL -> RB8                                                                 */
/*                                                                            */
/* LED                                                                        */
/* LED1 -> RC6                                                                */
/* LED2 -> RC7                                                                */
/* LED3 -> RC8                                                                */
/* LED4 -> RC9                                                                */
/*                                                                            */
/* GPIO                                                                       */
/* GP1 -> RC0 (CN8)                                                           */
/* GP2 -> RC1 (CN9)                                                           */
/* GP3 -> RC2 (CN10)                                                          */
/* GP4 -> RC3 (CN28)                                                          */
/* GP5 -> RA4 (CN0)                                                           */
/* GP6 -> RB4 (CN1)                                                           */
/* GP7 -> RB7 (CN23)                                                          */
/* GP8 -> RA8                                                                 */
/* HLT -> RA9                                                                 */
/******************************************************************************/

/******************************************************************************/
/* User Level #define Macros                                                  */
/******************************************************************************/
#ifdef UNAV_V1
    #define LED1 _LATC6              // Led 1 green
    #define LED2 _LATC7              // Led 2 green
    #define LED3 _LATC8              // Led 3 yellow
    #define LED4 _LATC9              // Led 4 red

    #define MOTOR_ENABLE1 _LATA7     // Enable Motore 1
    #define MOTOR_ENABLE2 _LATA10    // Enable Motore 2
#elif ROBOCONTROLLER_V3
    #define LED1 _LATA8              // Led 1 green
    #define LED2 _LATA9              // Led 2 green

    #define MOTOR_ENABLE1 _LATA1     // Enable Motore 1
    #define MOTOR_ENABLE2 _LATA4    // Enable Motore 2
#elif MOTION_CONTROL
    #define LED1 _LATA4              // Led Blu

    #define MOTOR_ENABLE1 _LATB2    // Enable Motore 1
    #define MOTOR_ENABLE2 _LATB3    // Enable Motore 2
#endif

    /** Definition for user interrupt **/
    #define PID_FLAG IFS0bits.OC1IF
    #define PARSER_FLAG IFS0bits.OC2IF
    #define DEAD_RECKONING_FLAG IFS3bits.RTCIF

    #define SGN(x)  ( ((x) < 0) ?  -1 : ( ((x) == 0 ) ? 0 : 1) )

/******************************************************************************/
/* User Function Prototypes                                                   */
/******************************************************************************/

    /**
     * I/O and Peripheral Initialization
     */
    void InitApp(void);
    /**
     * Protected memcpy, stop particular interrupt and copy data.
     * @param reg Interrupt to disable
     * @param destination data
     * @param source data
     * @param num size of data
     */
    void protectedMemcpy(unsigned reg, void *destination, const void *source, size_t num);

    /**
     * Evaluate max value of array on float
     * @param myArray array to find max value
     * @param size size of array
     * @return max value on array
     */
    int maxValue(float myArray[], size_t size);

#ifdef	__cplusplus
}
#endif

#endif	/* USER_H */