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

#include <serial/or_message.h>
#include <serial/or_frame.h>
#include "communication/serial.h"

#include "motors/motor_control.h"
#include "high_control/manager.h"

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

ICdata ICinfo[NUM_MOTORS];

//From system.c
extern process_t default_process[2];
extern process_t motor_process[PROCESS_MOTOR_LENGTH];
extern process_t motion_process[PROCESS_MOTION_LENGTH];

//From high_level_control
extern volatile unsigned int control_state;

////From user
//extern led_control_t led_controller[LED_NUM];
//extern bool led_effect;

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
/* 08 |      _INT0Interrupt     | 47 | N.A. _IC5Interrupt                     */
/* 09 | USED _IC1Interrupt      | 48 | N.A. _IC6Interrupt                     */
/* 10 | USED _OC1Interrupt      | 49 | N.A. _OC5Interrupt                     */
/* 11 | USED _T1Interrupt       | 50 | N.A. _OC6Interrupt                     */
/* 12 | USED _DMA0Interrupt     | 51 | N.A. _OC7Interrupt                     */
/* 13 | USED _IC2Interrupt      | 52 | N.A. _OC8Interrupt                     */
/* 14 | USED _OC2Interrupt      | 53 |      _PMPInterrupt                     */
/* 15 | USED _T2Interrupt       | 54 |      _DMA4Interrupt                    */
/* 16 |      _T3Interrupt       | 55 | N.A. _T6Interrupt                      */
/* 17 |      _SPI1ErrInterrupt  | 56 | N.A. _T7Interrupt                      */
/* 18 |      _SPI1Interrupt     | 57 | N.A. _SI2C2Interrupt                   */
/* 19 | USED _U1RXInterrupt     | 58 | N.A. _MI2C2Interrupt                   */
/* 20 |  X   _U1TXInterrupt     | 59 | N.A. _T8Interrupt                      */
/* 21 |  X   _ADC1Interrupt     | 60 | N.A. _T9Interrupt                      */
/* 22 | USED _DMA1Interrupt     | 61 | N.A. _INT3Interrupt                    */
/* 23 | N.A. _Interrupt         | 62 | N.A. _INT4Interrupt                    */
/* 24 |      _SI2C1Interrupt    | 63 | N.A. _C2RxRdyInterrupt                 */
/* 25 |      _MI2C1Interrupt    | 64 | N.A. _C2Interrupt                      */
/* 26 |      _CM1Interrupt      | 65 |      _PWM1Interrupt                    */
/* 27 |      _CNInterrupt       | 66 |      _QEI1Interrupt                    */
/* 28 |      _INT1Interrupt     | 67 | N.A. _DCIErrInterrupt                  */
/* 29 | N.A. _ADC2Interrupt     | 68 | N.A. _DCIInterrupt                     */
/* 30 |      _IC7Interrupt      | 69 |      _DMA5Interrupt                    */
/* 31 |      _IC8Interrupt      | 70 | USED _RTCCInterrupt                    */
/* 32 |      _DMA2Interrupt     | 71 |      _PWM1FaultInterrupt               */
/* 33 | USED _OC3Interrupt      | 72 | N.A. _Interrupt                        */
/* 34 |      _OC4Interrupt      | 73 |      _U1ErrInterrupt                   */
/* 35 |      _T4Interrupt       | 74 |      _U1ErrInterrupt                   */
/* 36 |      _T5Interrupt       | 75 |      _CRCInterrupt                     */
/* 37 |      _INT2Interrupt     | 76 |      _DMA6Interrupt                    */
/* 38 |      _U2RXInterrupt     | 77 |      _DMA7Interrupt                    */
/* 39 |      _U2TXInterrupt     | 78 |      _C1TxReqInterrupt                 */
/* 40 |      _SPI2ErrInterrupt  | 79 | N.A. _C2TxReqInterrupt                 */
/* 41 |      _SPI2Interrupt     | 80 | N.A. _Interrupt                        */
/* 42 |      _C1RxRdyInterrupt  | 81 |      _PWM2Interrupt                    */
/* 43 |      _C1Interrupt       | 82 |      _PWM2FaultInterrupt               */
/* 44 |      _DMA3Interrupt     | 83 |      _QEI2Interrupt                    */
/* 45 | N.A. _IC3Interrupt      | 84 | N.A. _Interrupt                        */
/* 46 | N.A. _IC4Interrupt      | 85 | N.A. _Interrupt                        */
/*                              | 86 |      _DAC1RInterrupt                   */
/*                              | 87 |      _DAC1LInterrupt                   */
/*                                                                            */
/* For alternate interrupt vector naming, simply add 'Alt' between the prim.  */
/* interrupt vector name '_' and the first character of the primary interrupt */
/* vector name.                                                               */
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
    t2 = IC1BUF; // IC1BUF is a FIFO, each reading is a POP
    t1 = IC1BUF;

    ICinfo[MOTOR_ZERO].timePeriod += ICinfo[MOTOR_ZERO].overTmr * PR2 + t2 - t1; // PR2 is 0xFFFF
    ICinfo[MOTOR_ZERO].overTmr = 0;

    (QEI1CONbits.UPDN ? ICinfo[MOTOR_ZERO].SIG_VEL++ : ICinfo[MOTOR_ZERO].SIG_VEL--); //Save sign Vel motor 0
//    ICinfo[MOTOR_ZERO].SIG_VEL = (QEI1CONbits.UPDN ? 1 : -1); //Save sign Vel L
    IFS0bits.IC1IF = 0;
}

void __attribute__((interrupt, auto_psv, shadow)) _IC2Interrupt(void) {
    unsigned int t1, t2;
    t2 = IC2BUF; // IC1BUF is a FIFO, each reading is a POP
    t1 = IC2BUF;

    ICinfo[MOTOR_ONE].timePeriod += ICinfo[MOTOR_ONE].overTmr * PR2 + t2 - t1; // PR2 is 0xFFFF
    ICinfo[MOTOR_ONE].overTmr = 0;
    
    //	if(QEI2CONbits.UPDN) SIG_VELR++;		//Save sign Vel R
    //	else SIG_VELR--;
    (QEI2CONbits.UPDN ? ICinfo[MOTOR_ONE].SIG_VEL++ : ICinfo[MOTOR_ONE].SIG_VEL--); //Save sign Vel motor 1
//    ICinfo[MOTOR_ONE].SIG_VEL = (QEI2CONbits.UPDN ? 1 : -1); //Save sign Vel R
    IFS0bits.IC2IF = 0;
}



void __attribute__((interrupt, auto_psv, shadow)) _T2Interrupt(void) {
    IFS0bits.T2IF = 0; // interrupt flag reset
    if (ICinfo[MOTOR_ZERO].timePeriod)
        ICinfo[MOTOR_ZERO].overTmr++; // timer overflow counter for Left engines
    if (ICinfo[MOTOR_ONE].timePeriod)
        ICinfo[MOTOR_ONE].overTmr++; // timer overflow counter for Right engines
}

//void __attribute__((interrupt, auto_psv)) _OC1Interrupt(void) {
//    motor_process[PROCESS_VELOCITY].time = MotorTaskController();
//    FLAG_TASK_MOTORS = 0; // interrupt flag reset
//}
//
//void __attribute__((interrupt, auto_psv)) _OC2Interrupt(void) {
//    default_process[PROCESS_PARSE].time = parse_packet();
//    PARSER_FLAG = 0; //interrupt flag reset
//}
//
//void __attribute__((interrupt, auto_psv)) _OC3Interrupt(void) {
//    //Will be added in feature #39
//    //time.process[PROCESS_MEASURE_VEL] = measureVelocity(REF_MOTOR_LEFT);
//    //time.process[PROCESS_MEASURE_VEL] += measureVelocity(REF_MOTOR_RIGHT);
//    MEASURE_FLAG = 0;
//}
//
//void __attribute__((interrupt, auto_psv)) _RTCCInterrupt(void) {
//    HighLevelTaskController();
//    FLAG_TASK_HIGH_LEVEL = 0; //interrupt flag reset
//}

void __attribute__((interrupt, auto_psv)) _DMA0Interrupt(void) {
    IFS0bits.DMA0IF = 0; // Clear the DMA0 Interrupt Flag
    adc_motors_current(); // Esecution mean value for current motors
}

void __attribute__((interrupt, auto_psv)) _DMA1Interrupt(void) {
    IFS0bits.DMA1IF = 0; // Clear the DMA1 Interrupt Flag
}
