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

#include <stdint.h>          /* For uint16_t definition                       */
#include <stdbool.h>         /* For true/false definition                     */
#include <dsp.h>             /* For DSP functionality                         */
#include <string.h>
#include <assert.h>
#include "system/user.h"     /* variables/params used by user.c               */
#include "system/system.h"

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

/******************************************************************************/
/* User Functions                                                             */
/******************************************************************************/

void InitApp(void) {

    // Peripheral PIN remapping
    //*************************************************************
    // Unlock Registers
    //*************************************************************
    asm volatile ( "mov #OSCCONL, w1 \n"
                "mov #0x45, w2 \n"
                "mov #0x57, w3 \n"
                "mov.b w2, [w1] \n"
                "mov.b w3, [w1] \n"
                "bclr OSCCON, #6 ");


#ifdef UNAV
    #warning -- compiling for uNav board : Set USER.H **************************
    // Input capture
    //***************************
    // Assign IC1 To Pin RP10
    //***************************
    RPINR7bits.IC1R = 10;
    //***************************
    // IC2 To Pin RP6
    //***************************
    RPINR7bits.IC2R = 6;

    // QEI
    //***************************
    // QEA1 To Pin RP10
    //***************************
    RPINR14bits.QEA1R = 10;
    //***************************
    // QEB1 To Pin RP11
    //***************************
    RPINR14bits.QEB1R = 11;
    //***************************
    // QEA2 To Pin RP5
    //***************************
    RPINR16bits.QEA2R = 5;
    //***************************
    // QEB2 To Pin RP6
    //***************************
    RPINR16bits.QEB2R = 6;

    // UART
    //***************************
    // Assign U2RX To Pin RP3, CTS tied Vss
    //***************************
    RPINR19bits.U2RXR = 3;
    RPINR19bits.U2CTSR = 0x1f;
    //***************************
    // Assign U2Tx To Pin RP2
    //***************************
    RPOR1bits.RP2R = 5;

    //***************************
    // Assign U1RX To Pin RP21, CTS tied Vss
    //***************************
    RPINR18bits.U1RXR = 21;
    RPINR18bits.U1CTSR = 0x1f;
    //***************************
    // Assign U1Tx To Pin RP20
    //***************************
    RPOR10bits.RP20R = 3;

    //*************************************************************
    // Lock Registers
    //*************************************************************
    asm volatile ( "mov #OSCCONL, w1 \n"
                "mov #0x45, w2 \n"
                "mov #0x57, w3 \n"
                "mov.b w2, [w1] \n"
                "mov.b w3, [w1] \n"
                "bset OSCCON, #6");
    // *********************************** Peripheral PIN selection

    /* Setup port direction */

    // weak pullups enable
    CNPU1 = 0xffff;
    CNPU2 = 0xffff;

    // led
    _TRISC6 = 0; //Led1
    _TRISC7 = 0; //Led2
    _TRISC8 = 0; //Led3
    _TRISC9 = 0; //Led4

    // encoder
    _TRISB10 = 1;
    _TRISB11 = 1;
    _TRISB6 = 1;
    _TRISB5 = 1;

    // H bridge
    _TRISA7 = 0; //Enable - Motor 1
    _TRISA10 = 0; //Enable - Motor 2
    _TRISB12 = 0; // PWM1 +
    _TRISB12 = 0; // PWM1 -
    _TRISB12 = 0; // PWM2 +
    _TRISB12 = 0; // PWM2 -

    // GPIO
    _TRISC0 = 1; // GPIO1
    _TRISC1 = 1; // GPIO2
    _TRISC2 = 1; // GPIO3
    _TRISC3 = 1; // GPIO4
    _TRISA4 = 1; // GPIO5
    _TRISB4 = 1; // GPIO6
    _TRISB7 = 1; // GPIO7
    _TRISA8 = 1; // GPIO8
    _TRISA9 = 1; // HALT

    // ADC
    _TRISA0 = 1; // CH1
    _TRISA1 = 1; // CH2
    _TRISB0 = 1; // CH3
    _TRISB1 = 1; // CH4

    /* Initialize peripherals */ // da controllare e adattare
    LED1 = 0;
    LED2 = 0;
    LED3 = 0;
    LED4 = 0;
#endif

#ifdef ROBOCONTROLLER_V3
    #warning -- compiling for RoboController V3 board : Set USER.H *************
    // Input capture
    //***************************
    // Assign IC1 To Pin RP10
    //***************************
    RPINR7bits.IC1R = 10;
    //***************************
    // IC2 To Pin RP6
    //***************************
    RPINR7bits.IC2R = 6;

    // QEI
    //***************************
    // QEA1 To Pin RP10
    //***************************
    RPINR14bits.QEA1R = 10;
    //***************************
    // QEB1 To Pin RP11
    //***************************
    RPINR14bits.QEB1R = 11;
    //***************************
    // QEA2 To Pin RP5
    //***************************
    RPINR16bits.QEA2R = 5;
    //***************************
    // QEB2 To Pin RP6
    //***************************
    RPINR16bits.QEB2R = 6;

    // UART
    //***************************
    // Assign U2RX To Pin RP3, CTS tied Vss
    //***************************
    RPINR19bits.U2RXR = 3;
    RPINR19bits.U2CTSR = 0x1f;
    //***************************
    // Assign U2Tx To Pin RP2
    //***************************
    RPOR1bits.RP2R = 5;

    //***************************
    // Assign U1RX To Pin RP21, CTS tied Vss
    //***************************
    RPINR18bits.U1RXR = 21;
    RPINR18bits.U1CTSR = 0x1f;
    //***************************
    // Assign U1Tx To Pin RP20
    //***************************
    RPOR10bits.RP20R = 3;

    //*************************************************************
    // Lock Registers
    //*************************************************************
    asm volatile ( "mov #OSCCONL, w1 \n"
                "mov #0x45, w2 \n"
                "mov #0x57, w3 \n"
                "mov.b w2, [w1] \n"
                "mov.b w3, [w1] \n"
                "bset OSCCON, #6");
    // *********************************** Peripheral PIN selection

    /* Setup port direction */

    // weak pullups enable
    CNPU1 = 0xffff;
    CNPU2 = 0xffff;

    // led
    _TRISC6 = 0; //Led1
    _TRISC7 = 0; //Led2
    _TRISC8 = 0; //Led3
    _TRISC9 = 0; //Led4

    // encoder
    _TRISB10 = 1;
    _TRISB11 = 1;
    _TRISB6 = 1;
    _TRISB5 = 1;

    // H bridge
    _TRISA7 = 0; //Enable - Motor 1
    _TRISA10 = 0; //Enable - Motor 2
    _TRISB12 = 0; // PWM1 +
    _TRISB12 = 0; // PWM1 -
    _TRISB12 = 0; // PWM2 +
    _TRISB12 = 0; // PWM2 -

    // GPIO
    _TRISC0 = 1; // GPIO1
    _TRISC1 = 1; // GPIO2
    _TRISC2 = 1; // GPIO3
    _TRISC3 = 1; // GPIO4
    _TRISA4 = 1; // GPIO5
    _TRISB4 = 1; // GPIO6
    _TRISB7 = 1; // GPIO7
    _TRISA8 = 1; // GPIO8
    _TRISA9 = 1; // HALT

    // ADC
    _TRISA0 = 1; // CH1
    _TRISA1 = 1; // CH2
    _TRISB0 = 1; // CH3
    _TRISB1 = 1; // CH4

    /* Initialize peripherals */ // da controllare e adattare
    LED1 = 0;
    LED2 = 0;
    LED3 = 0;
    LED4 = 0;
#endif

    /* Peripherical initalization */
    InitPWM();          //Open PWM
    InitQEI1();         //Open QEI1
    InitQEI2();         //Open QEI2
    InitIC1();          //Open Input Capture 1
    InitIC2();          //Open Input Capture 2
    InitTimer2();       //Open Timer2 for InputCapture 1 & 2
    InitADC();          //Open ADC for measure current motors
    InitDMA0();         //Open DMA0 for buffering measures ADC

    InitUART1();        //Open UART1 for serial comunication
    InitDMA1();         //Open DMA1 for Tx UART1

    InitTimer1();       //Open Timer1 for clock system
    InitInterrupts();   //Start others interrupts
}

void inline protectedMemcpy(unsigned reg, void *destination, const void *source, size_t num) {
    if (1 == reg) {
        reg = 0;
        memcpy(destination, source, num);
        reg = 1;
    } else {
        memcpy(destination, source, num);
    }
}

int maxValue(float myArray[], size_t size) {
    /* enforce the contract */
    //    assert(myArray && size);
    size_t i;
    float maxValue = myArray[0];

    for (i = 1; i < size; ++i) {
        if (myArray[i] > maxValue) {
            maxValue = myArray[i];
        }
    }
    return maxValue;
}