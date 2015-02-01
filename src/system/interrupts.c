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

#include <stdint.h>        /* Includes uint16_t definition   */
#include <stdbool.h>       /* Includes true/false definition */

#include "system/user.h"
#include "system/system.h"
#include "communication/serial.h"
#include "communication/parsing_messages.h"
#include "control/motors_PID.h"
#include "control/high_level_control.h"

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

unsigned int counter_odo = 0;
unsigned int counter_pid = 0;
volatile unsigned int overTmrL = 0;
volatile unsigned int overTmrR = 0;
volatile unsigned long timePeriodL = 0; //Periodo Ruota Sinistra
volatile unsigned long timePeriodR = 0; //Periodo Ruota Destra
volatile int SIG_VELL = 0; //Verso rotazione ruota sinistra
volatile int SIG_VELR = 0; //Verso rotazione ruota destra
volatile process_t time, priority, frequency;
process_buffer_t name_process_pid_l, name_process_pid_r, name_process_velocity, name_process_odometry;

//From user
extern led_control_t led_controller[LED_NUM];
extern bool led_effect;

/******************************************************************************/
/* Interrupt Vector Options                                                   */
/******************************************************************************/
/*                                                                            */
/* Refer to the C30 (MPLAB C Compiler for PIC24F MCUs and dsPIC33F DSCs) User */
/* Guide for an up to date list of the available interrupt options.           */
/* Alternately these names can be pulled from the device linker scripts.      */
/*                                                                            */
/* dsPIC33F Primary Interrupt Vector Names:                                   */
/*                                                                            */
/* _INT0Interrupt      _C1Interrupt                                           */
/* _IC1Interrupt       _DMA3Interrupt                                         */
/* _OC1Interrupt       _IC3Interrupt                                          */
/* _T1Interrupt        _IC4Interrupt                                          */
/* _DMA0Interrupt      _IC5Interrupt                                          */
/* _IC2Interrupt       _IC6Interrupt                                          */
/* _OC2Interrupt       _OC5Interrupt                                          */
/* _T2Interrupt        _OC6Interrupt                                          */
/* _T3Interrupt        _OC7Interrupt                                          */
/* _SPI1ErrInterrupt   _OC8Interrupt                                          */
/* _SPI1Interrupt      _DMA4Interrupt                                         */
/* _U1RXInterrupt      _T6Interrupt                                           */
/* _U1TXInterrupt      _T7Interrupt                                           */
/* _ADC1Interrupt      _SI2C2Interrupt                                        */
/* _DMA1Interrupt      _MI2C2Interrupt                                        */
/* _SI2C1Interrupt     _T8Interrupt                                           */
/* _MI2C1Interrupt     _T9Interrupt                                           */
/* _CNInterrupt        _INT3Interrupt                                         */
/* _INT1Interrupt      _INT4Interrupt                                         */
/* _ADC2Interrupt      _C2RxRdyInterrupt                                      */
/* _DMA2Interrupt      _C2Interrupt                                           */
/* _OC3Interrupt       _DCIErrInterrupt                                       */
/* _OC4Interrupt       _DCIInterrupt                                          */
/* _T4Interrupt        _DMA5Interrupt                                         */
/* _T5Interrupt        _U1ErrInterrupt                                        */
/* _INT2Interrupt      _U2ErrInterrupt                                        */
/* _U2RXInterrupt      _DMA6Interrupt                                         */
/* _U2TXInterrupt      _DMA7Interrupt                                         */
/* _SPI2ErrInterrupt   _C1TxReqInterrupt                                      */
/* _SPI2Interrupt      _C2TxReqInterrupt                                      */
/* _C1RxRdyInterrupt                                                          */
/*                                                                            */
/* dsPIC33E Primary Interrupt Vector Names:                                   */
/*                                                                            */
/* _INT0Interrupt     _IC4Interrupt      _U4TXInterrupt                       */
/* _IC1Interrupt      _IC5Interrupt      _SPI3ErrInterrupt                    */
/* _OC1Interrupt      _IC6Interrupt      _SPI3Interrupt                       */
/* _T1Interrupt       _OC5Interrupt      _OC9Interrupt                        */
/* _DMA0Interrupt     _OC6Interrupt      _IC9Interrupt                        */
/* _IC2Interrupt      _OC7Interrupt      _PWM1Interrupt                       */
/* _OC2Interrupt      _OC8Interrupt      _PWM2Interrupt                       */
/* _T2Interrupt       _PMPInterrupt      _PWM3Interrupt                       */
/* _T3Interrupt       _DMA4Interrupt     _PWM4Interrupt                       */
/* _SPI1ErrInterrupt  _T6Interrupt       _PWM5Interrupt                       */
/* _SPI1Interrupt     _T7Interrupt       _PWM6Interrupt                       */
/* _U1RXInterrupt     _SI2C2Interrupt    _PWM7Interrupt                       */
/* _U1TXInterrupt     _MI2C2Interrupt    _DMA8Interrupt                       */
/* _AD1Interrupt      _T8Interrupt       _DMA9Interrupt                       */
/* _DMA1Interrupt     _T9Interrupt       _DMA10Interrupt                      */
/* _NVMInterrupt      _INT3Interrupt     _DMA11Interrupt                      */
/* _SI2C1Interrupt    _INT4Interrupt     _SPI4ErrInterrupt                    */
/* _MI2C1Interrupt    _C2RxRdyInterrupt  _SPI4Interrupt                       */
/* _CM1Interrupt      _C2Interrupt       _OC10Interrupt                       */
/* _CNInterrupt       _QEI1Interrupt     _IC10Interrupt                       */
/* _INT1Interrupt     _DCIEInterrupt     _OC11Interrupt                       */
/* _AD2Interrupt      _DCIInterrupt      _IC11Interrupt                       */
/* _IC7Interrupt      _DMA5Interrupt     _OC12Interrupt                       */
/* _IC8Interrupt      _RTCCInterrupt     _IC12Interrupt                       */
/* _DMA2Interrupt     _U1ErrInterrupt    _DMA12Interrupt                      */
/* _OC3Interrupt      _U2ErrInterrupt    _DMA13Interrupt                      */
/* _OC4Interrupt      _CRCInterrupt      _DMA14Interrupt                      */
/* _T4Interrupt       _DMA6Interrupt     _OC13Interrupt                       */
/* _T5Interrupt       _DMA7Interrupt     _IC13Interrupt                       */
/* _INT2Interrupt     _C1TxReqInterrupt  _OC14Interrupt                       */
/* _U2RXInterrupt     _C2TxReqInterrupt  _IC14Interrupt                       */
/* _U2TXInterrupt     _QEI2Interrupt     _OC15Interrupt                       */
/* _SPI2ErrInterrupt  _U3ErrInterrupt    _IC15Interrupt                       */
/* _SPI2Interrupt     _U3RXInterrupt     _OC16Interrupt                       */
/* _C1RxRdyInterrupt  _U3TXInterrupt     _IC16Interrupt                       */
/* _C1Interrupt       _USB1Interrupt     _ICDInterrupt                        */
/* _DMA3Interrupt     _U4ErrInterrupt    _PWMSpEventMatchInterrupt            */
/* _IC3Interrupt      _U4RXInterrupt     _PWMSecSpEventMatchInterrupt         */
/*                                                                            */
/* For alternate interrupt vector naming, simply add 'Alt' between the prim.  */
/* interrupt vector name '_' and the first character of the primary interrupt */
/* vector name.  There is no Alternate Vector or 'AIVT' for the 33E family.   */
/*                                                                            */
/* For example, the vector name _ADC2Interrupt becomes _AltADC2Interrupt in   */
/* the alternate vector table.                                                */
/*                                                                            */
/* Example Syntax:                                                            */
/*                                                                            */
/* void __attribute__((interrupt,auto_psv)) <Vector Name>(void)               */
/* {                                                                          */
/*     <Clear Interrupt Flag>                                                 */
/* }                                                                          */
/*                                                                            */
/* For more comprehensive interrupt examples refer to the C30 (MPLAB C        */
/* Compiler for PIC24 MCUs and dsPIC DSCs) User Guide in the                  */
/* <C30 compiler instal directory>/doc directory for the latest compiler      */
/* release.  For XC16, refer to the MPLAB XC16 C Compiler User's Guide in the */
/* <XC16 compiler instal directory>/doc folder.                               */
/*                                                                            */
/******************************************************************************/
/* Interrupt Routines                                                         */

/******************************************************************************/

void __attribute__((interrupt, auto_psv, shadow)) _IC1Interrupt(void) {
    unsigned int t1, t2;
    t2 = IC1BUF;    // IC1BUF is a FIFO, each reading is a POP
    t1 = IC1BUF;
    IFS0bits.IC1IF = 0;
    //timePeriodL = overTmrL * PR2 + t2 - t1; // PR2 is 0xFFFF
    //overTmrL = 0;

    //	if(QEI1CONbits.UPDN) SIG_VELL++;		//Save sign Vel L
    //	else SIG_VELL--;
    //	if(t2>t1)
    //		timePeriodL = t2 - t1;
    //	else
    //		timePeriodL = (PR2 - t1) + t2;

    //SIG_VELL = (QEI1CONbits.UPDN ? 1 : -1); //Save sign Vel L
    (QEI1CONbits.UPDN ? SIG_VELL++ : SIG_VELL--); //Save sign Vel L

    if (overTmrL == 0) // TMR2 overflowed?
    {// see Microchip AN545
        timePeriodL += (t2 - t1);
    }
    else
    {// [7a]
        timePeriodL += (t2 + (PR2 - t1)
          +(PR2 * (overTmrL - 1)));

        overTmrL = 0;
    }
}

void __attribute__((interrupt, auto_psv, shadow)) _IC2Interrupt(void) {
    unsigned int t1, t2;
    t2 = IC2BUF;    // IC1BUF is a FIFO, each reading is a POP
    t1 = IC2BUF;
    IFS0bits.IC2IF = 0;
    timePeriodR = overTmrR * PR2 + t2 - t1; // PR2 is 0xFFFF
    overTmrR = 0;
    //	if(QEI2CONbits.UPDN) SIG_VELR++;		//Save sign Vel R
    //	else SIG_VELR--;
    //	if(t2>t1)
    //		timePeriodR = t2 - t1;
    //	else
    //		timePeriodR = (PR2 - t1) + t2;
    //Encoder speculare rispetto all'altro. Segni invertiti
    SIG_VELR = (QEI2CONbits.UPDN ? 1 : -1); //Save sign Vel R
}

void __attribute__((interrupt, auto_psv)) _T1Interrupt(void) {
    IFS0bits.T1IF = 0; // Clear Timer 1 Interrupt Flag?
    int led_counter = 0;

    if (counter_pid >= frequency.process[PROCESS_PID_LEFT]) {
        PID_FLAG = 1; //Start OC1Interrupt for PID control
        counter_pid = 0;
    }
    if (counter_odo >= frequency.process[PROCESS_ODOMETRY]) {
        DEAD_RECKONING_FLAG = 1;
        counter_odo = 0;
    }
    /**
     * Blink controller for all leds
     */
    for (led_counter = 0; led_counter < LED_NUM; led_counter++) {
        if (led_controller[led_counter].number_blink > LED_OFF)
            BlinkController(&led_controller[led_counter]);
    }
    counter_pid++;
    counter_odo++;
}

void __attribute__((interrupt, auto_psv, shadow)) _T2Interrupt(void) {
    IFS0bits.T2IF = 0; // interrupt flag reset
    if (timePeriodL)
    overTmrL++; // timer overflow counter for Left engines
    if (timePeriodR)
    overTmrR++; // timer overflow counter for Right engines
}

void __attribute__((interrupt, auto_psv)) _OC1Interrupt(void) {
    PID_FLAG = 0; // interrupt flag reset
    time.process[PROCESS_VELOCITY] = MotorTaskController();
    time.process[PROCESS_PID_LEFT] = MotorPIDL();
    time.process[PROCESS_PID_RIGHT] = MotorPIDR();
}

void __attribute__((interrupt, auto_psv)) _OC2Interrupt(void) {
    PARSER_FLAG = 0; //interrupt flag reset
    time.parse_packet = parse_packet();
}

void __attribute__((interrupt, auto_psv)) _RTCCInterrupt(void) {
    DEAD_RECKONING_FLAG = 0; //interrupt flag reset
    time.process[PROCESS_ODOMETRY] = deadReckoning();
}

unsigned int ReadUART1(void) {
    if (U1MODEbits.PDSEL == 3)
        return (U1RXREG);
    else
        return (U1RXREG & 0xFF);
}

void __attribute__((interrupt, auto_psv)) _U1RXInterrupt(void) {
    IFS0bits.U1RXIF = 0; // clear RX interrupt flag

    /* get the data */
    if (U1STAbits.URXDA == 1) {
        if (decode_pkgs(ReadUART1())) {
            PARSER_FLAG = 1; //if correct packet parse command start interrupt flag
        }
    } else {
        /* check for receive errors */
        if (U1STAbits.FERR == 1) {
            pkg_error(ERROR_FRAMMING);
        }
        /* must clear the overrun error to keep uart receiving */
        if (U1STAbits.OERR == 1) {
            U1STAbits.OERR = 0;
            pkg_error(ERROR_OVERRUN);
        }
    }
}

void __attribute__((interrupt, auto_psv)) _DMA0Interrupt(void) {
    IFS0bits.DMA0IF = 0; // Clear the DMA0 Interrupt Flag
    adc_motors_current(); // Esecution mean value for current motors
}

void __attribute__((interrupt, auto_psv)) _DMA1Interrupt(void) {
    IFS0bits.DMA1IF = 0; // Clear the DMA1 Interrupt Flag
}
