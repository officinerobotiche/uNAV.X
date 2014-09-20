/**********************************************************************
 * © 2014 µNAV crew
 *
 * GPIO remapping
 * Hardware init
 *
 * v 0.5 alpha 20/09/2014
 *
 **********************************************************************/
/*
RA4  LED 1
RB4  LED 2

RA0  AN0 current sensing 1 (Vref+ = Avdd = 3.3V)
RA1  AN1 current sensing 2 (Vref+ = Avdd = 3.3V)

RB0  AN2 Temperature M2 (PGD) (Ruota Sx)
RB1  AN3 Temperature M1 (PGC) (Ruota Dx)

RP5  RB5  QEA1
RP6  RB6  QEA2
RP10 RB7  QEB1
RP11 RB8  QEB2

RP7  RB7  U1RX
RP8	 RB8  U1TX

RP2  RB2  U2TX	(XBEE o 485)
RP3  RB3  U1RX	(XBEE o 485)

RP9  RB9  DIR

RP10 RB10 PWM-E1
RP11 RB11 PWM-E2

RP12 RB12 PWM2-H
RP13 RB13 PWM2-L
RP14 RB14 PWM1-H
RP15 RB15 PWM1-L

 */

/*

    LATA = 0x0000;
    LATB = 0x0000;
    TRISA = 0b1111111111101111;
    TRISB = 0b0000111111101111;
 
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
#include "user.h"            /* variables/params used by user.c               */
#include "system.h"

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

/******************************************************************************/
/* User Functions                                                             */
/******************************************************************************/

/* <Initialize variables in user.h and insert code for user algorithms.> */

void InitApp(void) {

    // Peripheral PIN remapping
    // Unlock Registers
    //*************************************************************
    asm volatile ( "mov #OSCCONL, w1 \n"
                "mov #0x45, w2 \n"
                "mov #0x57, w3 \n"
                "mov.b w2, [w1] \n"
                "mov.b w3, [w1] \n"
                "bclr OSCCON, #6 ");

    //***************************
    // Assign IC1 To Pin RP10
    //***************************
    RPINR7bits.IC1R = 10;

    //***************************
    // IC2 To Pin RP6
    //***************************
    RPINR7bits.IC2R = 6;

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


    //***************************
    // Assign U1RX To Pin RP3, CTS tied Vss
    //***************************
    RPINR18bits.U1RXR = 3;
    RPINR18bits.U1CTSR = 0x1f;

    //***************************
    // Assign U1Tx To Pin RP2
    //***************************
    RPOR1bits.RP2R = 3;

    //***************************
    // Assign U2RX To Pin RP7, CTS tied Vss
    //***************************
    RPINR19bits.U2RXR = 7;
    RPINR19bits.U2CTSR = 0x1f;

    //***************************
    // Assign U2Tx To Pin RP8
    //***************************
    RPOR4bits.RP8R = 5;

    //************************************************************
    // Lock Registers
    //************************************************************
    asm volatile ( "mov #OSCCONL, w1 \n"
                "mov #0x45, w2 \n"
                "mov #0x57, w3 \n"
                "mov.b w2, [w1] \n"
                "mov.b w3, [w1] \n"
                "bset OSCCON, #6");
    // *********************************** Peripheral PIN selection


    /* Setup analog functionality and port direction */
    _TRISA4 = 0; //Led
    _TRISB2 = 0; //Enable - Motor 1
    _TRISB3 = 0; //Enable - Motor 2
    _TRISB5 = 1;
    _TRISB6 = 1;
    _TRISB10 = 1;
    _TRISB11 = 1;
    // TODO Add analog functionality for ADC

    /* Initialize peripherals */
    LED = 0;
    InitPWM(); //Open PWM
    InitQEI1(); //Open QEI1
    InitQEI2(); //Open QEI2
    InitIC1(); //Open Input Capture 1
    InitIC2(); //Open Input Capture 2
    InitTimer2(); //Open Timer2 for InputCapture 1 & 2
    InitADC(); //Open ADC for measure current motors
    InitDMA0(); //Open DMA0 for buffering measures ADC

    InitUART1(); //Open UART1 for serial comunication
    InitDMA1(); //Open DMA1 for Tx UART1

    InitTimer1(); //Open Timer1 for clock system
    InitInterrupts(); //Start others interrupts
}

/* Protected Memcpy */
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