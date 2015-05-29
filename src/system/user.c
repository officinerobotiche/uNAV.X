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

//led_control_t led_controller[LED_NUM];
//bool led_effect = false;
//bool first = true;
//short load_blink[LED_NUM];
//pin_t led_1 = {&LED1_PORT, LED1_NUM};
//#if defined(UNAV_V1) || defined(ROBOCONTROLLER_V3)
//pin_t led_2 = {&LED2_PORT, LED2_NUM};
//#endif
//#if defined(UNAV_V1)
//pin_t led_3 = {&LED3_PORT, LED3_NUM};
//pin_t led_4 = {&LED4_PORT, LED4_NUM};
//#endif

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
#ifdef UNAV_V1
    // Input capture
    RPINR7bits.IC1R = 10; // IC1 To Pin RP10
    RPINR7bits.IC2R = 6; // IC2 To Pin RP6
    // QEI
    RPINR14bits.QEA1R = 10; // QEA1 To Pin RP10
    RPINR14bits.QEB1R = 11; // QEB1 To Pin RP11
    RPINR16bits.QEA2R = 5; // QEA2 To Pin RP5
    RPINR16bits.QEB2R = 6; // QEB2 To Pin RP6
    // UART
    RPINR18bits.U1RXR = 21; // U1RX To Pin RP21, CTS tied Vss
    RPINR18bits.U1CTSR = 0x1f;
    RPOR10bits.RP20R = 3; // U1Tx To Pin RP20

    RPINR19bits.U2RXR = 3; // U2RX To Pin RP3, CTS tied Vss
    RPINR19bits.U2CTSR = 0x1f;
    RPOR1bits.RP2R = 5; // U2Tx To Pin RP2
#elif ROBOCONTROLLER_V3
    // Input capture
    RPINR7bits.IC1R = 22; // IC1 To Pin RP22
    RPINR7bits.IC2R = 24; // IC2 To Pin RP24
    // QEI
    RPINR14bits.QEA1R = 22; // QEA1 To Pin RP22
    RPINR14bits.QEB1R = 23; // QEB1 To Pin RP23
    RPINR16bits.QEA2R = 24; // QEA2 To Pin RP24
    RPINR16bits.QEB2R = 25; // QEB2 To Pin RP25
    // UART
    RPINR18bits.U1RXR = 20; // U1RX To Pin RP20
    RPOR10bits.RP21R = 3;   // U1TX To Pin RP21

    //RPINR19bits.U2RXR = 6;  // U2RX To Pin RP6
    RPINR19bits.U2RXR = 8;    // U2RX To Pin RP8
    //RPOR2bits.RP5R = 5;     // U2TX To Pin RP5
    RPOR4bits.RP8R = 5;       // U2TX To Pin RP5
#elif MOTION_CONTROL
    // Input capture
    RPINR7bits.IC1R = 5; // Assign Input Capture 1 To Pin RP5
    RPINR7bits.IC2R = 10; // Assign Input Capture 2 To Pin RP10
    // QEI
    RPINR14bits.QEA1R = 5; // Assign QEA1 To Pin RP5
    RPINR14bits.QEB1R = 6; // Assign QEB1 To Pin RP6
    RPINR16bits.QEA2R = 11; // Assign QEA2 To Pin RP11
    RPINR16bits.QEB2R = 10; // Assign QEB2 To Pin RP10
    //UART RX
    RPINR18bits.U1RXR = 8; // Assign U1RX To Pin RP8
    RPOR4bits.RP9R = 3; // Assign U1Tx To Pin RP9
#else
#error Configuration error. Does not selected a board!
#endif
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
    CNPU2 = 0x9fff; // Pull up on CN29 and CN30 must not be enable to avoid problems with clock!!! by Walt

#ifdef UNAV_V1
    // LED
    _TRISC6 = 0; // LED 1 Green
    _TRISC7 = 0; // LED 2 Green
    _TRISC8 = 0; // LED 3 Yellow
    _TRISC9 = 0; // LED 4 Red
    // Encoders
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
#elif ROBOCONTROLLER_V3
    // LED
    _TRISA8 = 0; // LED1
    _TRISA9 = 0; // LED2
    // Encodes
    _TRISC6 = 1; // QEA_1
    _TRISC7 = 1; // QEB_1
    _TRISC8 = 1; // QEA_2
    _TRISC9 = 1; // QEB_2
    // H-Bridge
    _TRISA1 = 0; // MOTOR_EN1
    _TRISA4 = 0; // MOTOR_EN2
    // GPIO
    _TRISA7 = 0; // AUX1
    _TRISA10 = 0; // AUX2
    // ADC
    _TRISB2 = 1; // CH1
    _TRISB3 = 1; // CH2
    _TRISC0 = 1; // CH3
    _TRISC1 = 1; // CH4
    // Others
    _TRISB7 = 0; // DIR RS485 UART2
    _TRISB8 = 0; // SDA = Out : Connettore IC2 pin 6
    _TRISB9 = 0; // SCL = Out : Connettore IC2 pin 5
    _TRISB4 = 0; // RB4 = Out : Connettore IC2 pin 4
    _TRISC2 = 0; // OUT Float
    _TRISC3 = 0; // DIR RS485 UART1
#elif MOTION_CONTROL
    _TRISA4 = 0; //Led
    _TRISB2 = 0; //Enable - Motor 1
    _TRISB3 = 0; //Enable - Motor 2
    _TRISB5 = 1;
    _TRISB6 = 1;
    _TRISB10 = 1;
    _TRISB11 = 1;
#else
#error Configuration error. Does not selected a board!
#endif

    /* Peripherical initalization */
    InitTimer2(); //Open Timer2 for InputCapture 1 & 2
    InitADC(); //Open ADC for measure current motors
    InitDMA0(); //Open DMA0 for buffering measures ADC

    InitTimer1(); //Open Timer1 for clock system
    //InitInterrupts(); //Start others interrupts
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

//int maxValue(int* myArray, size_t size) {
//    /* enforce the contract */
//    //    assert(myArray && size);
//    size_t i;
//    int maxValue = myArray[0];
//
//    for (i = 1; i < size; ++i) {
//        if (myArray[i] > maxValue) {
//            maxValue = myArray[i];
//        }
//    }
//    return maxValue;
//}
//
//float maxValueFloat(float* myArray, size_t size) {
//    /* enforce the contract */
//    //    assert(myArray && size);
//    size_t i;
//    float maxValue = myArray[0];
//
//    for (i = 1; i < size; ++i) {
//        if (myArray[i] > maxValue) {
//            maxValue = myArray[i];
//        }
//    }
//    return maxValue;
//}

//void InitLed(void) {
//    int i;
//    led_controller[0].pin = &led_1;
//#if defined(UNAV_V1) || defined(ROBOCONTROLLER_V3)
//    led_controller[1].pin = &led_2;
//#endif
//#if defined(UNAV_V1)
//    led_controller[2].pin = &led_3;
//    led_controller[3].pin = &led_4;
//#endif
//    for (i = 0; i < LED_NUM; ++i) {
//        led_controller[i].CS_mask = 1 << led_controller[i].pin->CS_pin;
//        led_controller[i].wait = 0;
//        UpdateBlink(i, LED_OFF);
//    }
//}

//void UpdateBlink(short num, short blink) {
//    led_controller[num].number_blink = blink;
//    switch (led_controller[num].number_blink) {
//        case LED_OFF:
//            //Clear bit - Set to 0
//            *(led_controller[num].pin->CS_PORT) &= ~led_controller[num].CS_mask;
//            break;
//        case LED_ALWAYS_HIGH:
//            //Set bit - Set to 1
//            *(led_controller[num].pin->CS_PORT) |= led_controller[num].CS_mask;
//            break;
//        default:
//            led_controller[num].fr_blink = FRTMR1 / (2 * led_controller[num].number_blink);
//            break;
//    }
//    led_controller[num].counter = 0;
//}
//
///**
// * Tc -> counter = 1sec = 1000 interrupts
// * !       Tc/2        !   Tc/2       !
// * !     !_____   _____!              !
// * !     !|   |   |   |!              !
// * !-----!|   |---|   |! . . . -------!
// * !     !             !              !
// * ! WAIT   Tc/2-WAIT  !   Tc/2       !
// */
//
//inline void BlinkController(led_control_t *led) {
//    if (led->counter > led->wait && led->counter < FRTMR1) {
//        if (led->counter % led->fr_blink == 0) {
//            //Toggle bit
//            *(led->pin->CS_PORT) ^= led->CS_mask;
//        }
//        led->counter++;
//    } else if (led->counter >= 3 * FRTMR1 / 2) {
//        led->counter = 0;
//    } else {
//        //Clear bit - Set to 0
//        *(led->pin->CS_PORT) &= ~led->CS_mask;
//        led->counter++;
//    }
//}
//
//void blinkflush() {
//    int i;
//    for (i = 0; i < LED_NUM; ++i) {
//        led_controller[i].wait = i * ((float) FRTMR1 / LED_NUM);
//        load_blink[i] = led_controller[i].number_blink;
//        UpdateBlink(i, 1);
//    }
//    led_effect = true;
//}
//
//void EffectStop() {
//    int i;
//    int value = 0;
//    if (led_effect) {
//        for (i = 0; i < LED_NUM; ++i) {
//            value += led_controller[i].counter;
//        }
//        if (value == 0) {
//            if (~first) {
//                for (i = 0; i < LED_NUM; ++i) {
//                    UpdateBlink(i, load_blink[i]);
//                    led_controller[i].wait = 0;
//                }
//                led_effect = false;
//                first = true;
//            } else {
//                first = false;
//            }
//        }
//    }
//}