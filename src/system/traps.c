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

/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

#include <xc.h>             /* Device header file */

#include <stdint.h>        /* Includes uint16_t definition */
#include <stdbool.h>       /* Includes true/false definition */
#include <libpic30.h>      /* Includes for delay definition */

/******************************************************************************/
/* Trap Function Prototypes                                                   */
/******************************************************************************/

/* <Other function prototypes for debugging trap code may be inserted here>   */

/* Use if INTCON2 ALTIVT=1 */
void __attribute__((interrupt, no_auto_psv)) _OscillatorFail(void);
void __attribute__((interrupt, no_auto_psv)) _AddressError(void);
void __attribute__((interrupt, no_auto_psv)) _StackError(void);
void __attribute__((interrupt, no_auto_psv)) _MathError(void);

#if defined(__HAS_DMA__)

void __attribute__((interrupt, no_auto_psv)) _DMACError(void);

#endif

#if defined(__dsPIC33F__)

/* Use if INTCON2 ALTIVT=0 */
void __attribute__((interrupt, no_auto_psv)) _AltOscillatorFail(void);
void __attribute__((interrupt, no_auto_psv)) _AltAddressError(void);
void __attribute__((interrupt, no_auto_psv)) _AltStackError(void);
void __attribute__((interrupt, no_auto_psv)) _AltMathError(void);

#if defined(__HAS_DMA__)

void __attribute__((interrupt, no_auto_psv)) _AltDMACError(void);

#endif

#endif

/* Default interrupt handler */
void __attribute__((interrupt, no_auto_psv)) _DefaultInterrupt(void);

#if defined(__dsPIC33E__)

/* These are additional traps in the 33E family.  Refer to the PIC33E
migration guide.  There are no Alternate Vectors in the 33E family. */
void __attribute__((interrupt, no_auto_psv)) _HardTrapError(void);
void __attribute__((interrupt, no_auto_psv)) _SoftTrapError(void);

#endif

/******************************************************************************/
/* Trap Handling                                                              */
/*                                                                            */
/* These trap routines simply ensure that the device continuously loops       */
/* within each routine.  Users who actually experience one of these traps     */
/* can add code to handle the error.  Some basic examples for trap code,      */
/* including assembly routines that process trap sources, are available at    */
/* www.microchip.com/codeexamples                                             */
/******************************************************************************/

/**
 * Traps table
 *
 * *    TRAPS           F.ERR   LED1    LED2    LED3    LED4
 * * _OscillatorFail    1       blink   1       0       0
 * * _AltOscillatorFail 1       blink   1       0       0
 * * _AddressError      2       blink   0       1       0
 * * _AltAddressError   2       blink   0       1       0
 * * _StackError        3       blink   1       1       0
 * * _AltStackError     3       blink   1       1       0
 * * _MathError         4       blink   0       0       1
 * * _AltMathError      4       blink   0       0       1
 * * _DMACError         5       blink   1       0       1
 * * _AltDMACError      5       blink   1       0       1
 * * _DefaultInterrupt  6       blink   0       1       1
 *
 */

        // Current ADC buffer dimension
#ifdef UNAV_V1
/// Number of available LEDs
#define LED_NUM 4
/// LED 1 - Green
#define LED1_BIT _LATC6          // Led 1 Green
#define LED1 0                   // Led 1 Green
/// LED 2 - Red
#define LED2_BIT _LATC7          // Led 2 Red
#define LED2 1                   // Led 2 Red
/// LED 3 - Yellow
#define LED3_BIT _LATC8          // Led 3 Yellow
#define LED3 2                   // Led 3 Yellow
/// LED 4 - Blue
#define LED4_BIT _LATC9          // Led 4 Blue
#define LED4 3                   // Led 4 Blue
#elif ROBOCONTROLLER_V3
/// Number of available LEDs
#define LED_NUM 2
/// LED 1 - Green
#define LED1_BIT _LATA8          // Led 1 green
#define LED1 0                   // Led 1 green
/// LED 2 - Green
#define LED2_BIT _LATA9          // Led 2 green
#define LED2 1                   // Led 2 green
#elif MOTION_CONTROL
/// Number of available LEDs
#define LED_NUM 1
/// LED 1 - Green
#define LED1_BIT _LATA4          // Led Blue
#define LED1 0                   // Led Blue
#endif

void disable_routine()
{
    // Disable PWM
    PTCONbits.PTEN = 0;
}

/* Primary (non-alternate) address error trap function declarations */
void __attribute__((interrupt, no_auto_psv)) _OscillatorFail(void) {
    INTCON1bits.OSCFAIL = 0; /* Clear the trap flag */
    // Disable all peripheral enabled    
    disable_routine();
    // LED blink error
    while (1) {
        LED1_BIT = 1;
        __delay32(200000); // delay of 8 MHz RC oscillator
        LED1_BIT = 0;
        __delay32(200000);
        // fatal error 1
#ifdef UNAV_V1
        LED2_BIT = 1;
        LED3_BIT = 0;
        LED4_BIT = 0;
#elif ROBOCONTROLLER_V3
        LED2_BIT = 1;
#endif
    }
}

void __attribute__((interrupt, no_auto_psv)) _AddressError(void) {
    INTCON1bits.ADDRERR = 0; /* Clear the trap flag */
    // Disable all peripheral enabled    
    disable_routine();
    // LED blink error
    while (1) {
        LED1_BIT = 1;
        __delay32(2000000);
        LED1_BIT = 0;
        __delay32(2000000);
        // fatal error 2
#ifdef UNAV_V1
        LED2_BIT = 0;
        LED3_BIT = 1;
        LED4_BIT = 0;
#elif ROBOCONTROLLER_V3
        LED2_BIT = 0;
#endif
    };
}

void __attribute__((interrupt, no_auto_psv)) _StackError(void) {
    INTCON1bits.STKERR = 0; /* Clear the trap flag */
    // Disable all peripheral enabled    
    disable_routine();
    // LED blink error
    while (1) {
        LED1_BIT = 1;
        __delay32(2000000);
        LED1_BIT = 0;
        __delay32(2000000);
        // fatal error 3
#ifdef UNAV_V1
        LED2_BIT = 1;
        LED3_BIT = 1;
        LED4_BIT = 0;
#elif ROBOCONTROLLER_V3
        LED2_BIT = 1;
#endif
    };
}

void __attribute__((interrupt, no_auto_psv)) _MathError(void) {
    INTCON1bits.MATHERR = 0; /* Clear the trap flag */
    // Disable all peripheral enabled    
    disable_routine();
    // LED blink error
    while (1) {
        LED1_BIT = 1;
        __delay32(2000000);
        LED1_BIT = 0;
        __delay32(2000000);
        // fatal error 4
#ifdef UNAV_V1
        LED2_BIT = 0;
        LED3_BIT = 0;
        LED4_BIT = 1;
#elif ROBOCONTROLLER_V3
        LED2_BIT = 0;
#endif
    };
}

#if defined(__HAS_DMA__)

void __attribute__((interrupt, no_auto_psv)) _DMACError(void) {
    INTCON1bits.DMACERR = 0; /* Clear the trap flag */
    // Disable all peripheral enabled    
    disable_routine();
    // LED blink error
    while (1) {
        LED1_BIT = 1;
        __delay32(2000000);
        LED1_BIT = 0;
        __delay32(2000000);
        // fatal error 5
#ifdef UNAV_V1
        LED2_BIT = 1;
        LED3_BIT = 0;
        LED4_BIT = 1;
#elif ROBOCONTROLLER_V3
        LED2_BIT = 1;
#endif
    };
}

#endif

#if defined(__dsPIC33F__)

/* Alternate address error trap function declarations */
void __attribute__((interrupt, no_auto_psv)) _AltOscillatorFail(void) {
    INTCON1bits.OSCFAIL = 0; /* Clear the trap flag */
    // Disable all peripheral enabled    
    disable_routine();
    // LED blink error
    while (1) {
        LED1_BIT = 1;
        __delay32(2000000);
        LED1_BIT = 0;
        __delay32(2000000);
        // fatal error 1
#ifdef UNAV_V1
        LED2_BIT = 1;
        LED3_BIT = 0;
        LED4_BIT = 0;
#elif ROBOCONTROLLER_V3
        LED2_BIT = 1;
#endif
    };
}

void __attribute__((interrupt, no_auto_psv)) _AltAddressError(void) {
    INTCON1bits.ADDRERR = 0; /* Clear the trap flag */
    // Disable all peripheral enabled    
    disable_routine();
    // LED blink error
    while (1) {
        LED1_BIT = 1;
        __delay32(2000000);
        LED1_BIT = 0;
        __delay32(2000000);
        // fatal error 2
#ifdef UNAV_V1
        LED2_BIT = 0;
        LED3_BIT = 1;
        LED4_BIT = 0;
#elif ROBOCONTROLLER_V3
        LED2_BIT = 0;
#endif
    };
}

void __attribute__((interrupt, no_auto_psv)) _AltStackError(void) {
    INTCON1bits.STKERR = 0; /* Clear the trap flag */
    // Disable all peripheral enabled    
    disable_routine();
    // LED blink error
    while (1) {
        LED1_BIT = 1;
        __delay32(2000000);
        LED1_BIT = 0;
        __delay32(2000000);
        // fatal error 3
#ifdef UNAV_V1
        LED2_BIT = 1;
        LED3_BIT = 1;
        LED4_BIT = 0;
#elif ROBOCONTROLLER_V3
        LED2_BIT = 1;
#endif
    };
}

void __attribute__((interrupt, no_auto_psv)) _AltMathError(void) {
    INTCON1bits.MATHERR = 0; /* Clear the trap flag */
    // Disable all peripheral enabled    
    disable_routine();
    // LED blink error
    while (1) {
        LED1_BIT = 1;
        __delay32(2000000);
        LED1_BIT = 0;
        __delay32(2000000);
        // fatal error 4
#ifdef UNAV_V1
        LED2_BIT = 0;
        LED3_BIT = 0;
        LED4_BIT = 1;
#elif ROBOCONTROLLER_V3
        LED2_BIT = 0;
#endif
    };
}

#if defined(__HAS_DMA__)

void __attribute__((interrupt, no_auto_psv)) _AltDMACError(void) {
    INTCON1bits.DMACERR = 0; /* Clear the trap flag */
    // Disable all peripheral enabled    
    disable_routine();
    // LED blink error
    while (1) {
        LED1_BIT = 1;
        __delay32(2000000);
        LED1_BIT = 0;
        __delay32(2000000);
        // fatal error 5
#ifdef UNAV_V1
        LED2_BIT = 1;
        LED3_BIT = 0;
        LED4_BIT = 1;
#elif ROBOCONTROLLER_V3
        LED2_BIT = 1;
#endif
    };
}

#endif

#endif

/******************************************************************************/
/* Default Interrupt Handler                                                  */
/*                                                                            */
/* This executes when an interrupt occurs for an interrupt source with an     */
/* improperly defined or undefined interrupt handling routine.                */

/******************************************************************************/
void __attribute__((interrupt, no_auto_psv)) _DefaultInterrupt(void) {
    // Disable all peripheral enabled    
    disable_routine();
    // LED blink error
    while (1) {
        LED1_BIT = 1;
        __delay32(2000000);
        LED1_BIT = 0;
        __delay32(2000000);
        // fatal error 6
#ifdef UNAV_V1
        LED2_BIT = 0;
        LED3_BIT = 1;
        LED4_BIT = 1;
#elif ROBOCONTROLLER_V3
        LED2_BIT = 0;
#endif
    };
}

#if defined(__dsPIC33E__)

/* These traps are new to the dsPIC33E family.  Refer to the device Interrupt
chapter of the FRM to understand trap priority. */
void __attribute__((interrupt, no_auto_psv)) _HardTrapError(void) {
    while (1);
}

void __attribute__((interrupt, no_auto_psv)) _SoftTrapError(void) {
    while (1);
}

#endif
