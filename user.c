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
    // Unlock Registers  *****************************************
    asm volatile ( "mov #OSCCONL, w1 \n"
                "mov #0x45, w2 \n"
                "mov #0x57, w3 \n"
                "mov.b w2, [w1] \n"
                "mov.b w3, [w1] \n"
                "bclr OSCCON, #6 ");
    //**********************************************************//
    // Configure Input Functions
    //Encoder
    RPINR7bits.IC1R = 5; // Assign Input Capture 1 To Pin RP5
    RPINR7bits.IC2R = 10; // Assign Input Capture 2 To Pin RP10
    RPINR14bits.QEA1R = 5; // Assign QEA1 To Pin RP5
    RPINR14bits.QEB1R = 6; // Assign QEB1 To Pin RP6
    RPINR16bits.QEA2R = 11; // Assign QEA2 To Pin RP11
    RPINR16bits.QEB2R = 10; // Assign QEB2 To Pin RP10
    //UART RX
    RPINR18bits.U1RXR = 8; // Assign U1RX To Pin RP8

    // Configure Output Functions
    //UART TX
    RPOR4bits.RP9R = 3; // Assign U1Tx To Pin RP9

    //************************************************************
    // Lock Registers ********************************************
    asm volatile ( "mov #OSCCONL, w1 \n"
                "mov #0x45, w2 \n"
                "mov #0x57, w3 \n"
                "mov.b w2, [w1] \n"
                "mov.b w3, [w1] \n"
                "bset OSCCON, #6");
    //**********************************************************//

    /* Setup analog functionality and port direction */
    _TRISA4 = 0; //Led
    _TRISB2 = 0; //Enable - Motor 1
    _TRISB3 = 0; //Enable - Motor 2
    _TRISB5 = 1;
    _TRISB6 = 1;
    _TRISB10 = 1;
    _TRISB11 = 1;
    // TODO Add analog funcionality for ADC

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