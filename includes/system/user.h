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

    /**
     * Struct to control blink led
     * - port name to bit register to mount led
     * - counter to control blink led
     * - number of blink in a period, if:
     *      -# -1 fixed led
     *      -# 0 led off
     *      -# n number of blink
     */
    typedef struct led_control {
        volatile unsigned int * CS_PORT;
        unsigned int CS_pin;
        unsigned int counter;
        unsigned int wait;
        short number_blink;
    } led_control_t;

/******************************************************************************/
/* User Level #define Macros                                                  */
/******************************************************************************/
#ifdef UNAV_V1
    #define LED_NUM 4
    #define LED1 _LATC6              // Led 1 green
    #define LED1_PORT LATC           // Led 1 green
    #define LED1_NUM  6              // Led 1 green
    #define LED2 _LATC7              // Led 2 green
    #define LED2_PORT LATC           // Led 2 green
    #define LED2_NUM  7              // Led 2 green
    #define LED3 _LATC8              // Led 3 yellow
    #define LED3_PORT LATC           // Led 3 yellow
    #define LED3_NUM  8              // Led 3 yellow
    #define LED4 _LATC9              // Led 4 red
    #define LED4_PORT LATC           // Led 4 red
    #define LED4_NUM  9              // Led 1 green

    #define MOTOR_ENABLE1 _LATA7     // Enable Motore 1
    #define MOTOR_ENABLE2 _LATA10    // Enable Motore 2
#elif ROBOCONTROLLER_V3
    #define LED_NUM 2
    #define LED1 _LATA8              // Led 1 green
    #define LED1_PORT LATA           // Led 1 green
    #define LED1_NUM  8              // Led 1 green
    #define LED2 _LATA9              // Led 2 green
    #define LED2_PORT LATA           // Led 1 green
    #define LED2_NUM  9              // Led 1 green

    #define MOTOR_ENABLE1 _LATA1     // Enable Motore 1
    #define MOTOR_ENABLE2 _LATA4    // Enable Motore 2
#elif MOTION_CONTROL
    #define LED_NUM 1
    #define LED1 _LATA4              // Led Blu
    #define LED1_PORT LATA           // Led Blu
    #define LED1_NUM  4              // Led Blu

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

    /**
     * Blink control led
     * @param led to control
     */
    void BlinkController(led_control_t *led);

    void blinkflush();

#ifdef	__cplusplus
}
#endif

#endif	/* USER_H */