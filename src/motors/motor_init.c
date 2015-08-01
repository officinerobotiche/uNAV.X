/*
 * Copyright (C) 2015 Officine Robotiche
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
#include <pwm12.h>
#include <string.h>

#include "system/system.h"
#include "motors/motor_control.h"

#ifdef UNAV_V1
/// ENABLE 1
hardware_bit_t enable_1 = REGISTER_INIT(LATA, 7);
/// ENABLE 2
hardware_bit_t enable_2 = REGISTER_INIT(LATA, 10);
#elif ROBOCONTROLLER_V3
/// ENABLE 1
hardware_bit_t enable_1 = REGISTER_INIT(LATA, 1);
/// ENABLE 2
hardware_bit_t enable_2 = REGISTER_INIT(LATA, 4);
#elif MOTION_CONTROL
/// ENABLE 1
hardware_bit_t enable_1 = REGISTER_INIT(LATB, 2);
/// ENABLE 2
hardware_bit_t enable_2 = REGISTER_INIT(LATB, 3);
#endif

gpio_t enable[2];

/*****************************************************************************/
/* Global Variable Declaration                                               */
/*****************************************************************************/

// Dynamic Interrupt Capture
#define IC_MODE0    0b001 // 2X mode (default)
#define IC_MODE1    0b011 // 1X mode
#define IC_MODE2    0b100 // 1/4X mode
#define IC_MODE3    0b101 // 1/16X mode
#define IC_DISABLE  0b00
const int IcMode[4] = {IC_MODE0, IC_MODE1, IC_MODE2, IC_MODE3};

/*****************************************************************************/
/* User Functions                                                            */
/*****************************************************************************/

void InitPWM(void) {
    // Holds the value to be loaded into dutycycle register
    unsigned int period;
    // Holds the value to be loaded into special event compare register
    unsigned int sptime;
    // Holds PWM configuration value
    unsigned int config1;
    // Holds the value be loaded into PWMCON1 register
    unsigned int config2;
    // Holds the value to config the special event trigger postscale and dutycycle
    unsigned int config3;
    // Config PWM
    period = 2048; // PWM F=19,340Hz counting UP 12bit resolution @ Fcy=39.628 MHz
    sptime = 0x0;
    // 1:1 postscaler, 1:1 prescale, free running mode
    // PWM time base ON, count up
    config1 = PWM1_DIS & PWM1_IDLE_CON & PWM1_OP_SCALE1 & PWM1_IPCLK_SCALE1 &
            PWM1_MOD_FREE;
    // PWM1H e PWM1L enabled in complementar mode
    // dsPICs with 3 pairs of PWM pins have one timer only (A)
    config2 = PWM1_MOD1_COMP & PWM1_PEN1L & PWM1_PEN1H &
            PWM1_MOD2_COMP & PWM1_PEN2L & PWM1_PEN2H &
            PWM1_PDIS3H & PWM1_PDIS3L;
    config3 = PWM1_SEVOPS1 & PWM1_OSYNC_PWM & PWM1_UEN;
    OpenMCPWM1(period, sptime, config1, config2, config3);
    // Dead Time Unit A assigned to both 1 & 2 PWM pairs
    /* SetMCPWM1DeadTimeAssignment(PWM1_DTS1A_UA & PWM1_DTS1I_UA & PWM1_DTS2A_UA & PWM1_DTS2I_UA); */
    P1DTCON2bits.DTS1A = 0;
    P1DTCON2bits.DTS1I = 0;
    P1DTCON2bits.DTS2A = 0;
    P1DTCON2bits.DTS2I = 0;
    // Dead time 100ns = 0.2% of PWM period
    SetMCPWM1DeadTimeGeneration(PWM1_DTA4 & PWM1_DTAPS1);
    // dutycyclereg=1, dutycycle=50% (motore fermo in LAP mode , updatedisable=0
    SetDCMCPWM1(1, 2048, 0);
    SetDCMCPWM1(2, 2048, 0);

    ConfigIntMCPWM1(PWM1_INT_DIS);
}

void InitQEI(short motIdx) {
    switch (motIdx) {
        case MOTOR_ZERO:
            //QEI1CONbits.CNTERR= 0; // No position count error has occurred
            QEI1CONbits.QEISIDL = 1; // Discontinue module operation when device enters Idle mode
            QEI1CONbits.QEIM = 7; // Quadrature Encoder Interface enabled (x4 mode) with position counter reset by match (MAXxCNT)
            QEI1CONbits.SWPAB = (get_motor_parameters(MOTOR_ZERO).rotation >= 1) ? 1 : 0; // Phase A and Phase B inputs swapped
            QEI1CONbits.PCDOUT = 0; // Position counter direction status output disabled (Normal I/O pin operation)
            //QEI1CONbits.TQGATE= 0  // Timer gated time accumulation disabled
            //QEI1CONbits.TQCKPS = 0b00	// 1:1 prescale value
            QEI1CONbits.POSRES = 0; // Index pulse does not reset position counter

            DFLT1CONbits.QEOUT = 1; // Digital filter outputs enabled on QEAx/QEBx/INDXx pins
            DFLT1CONbits.QECK = 6; // 1:128 Clock divide for QEAx/QEBx/INDXx

            MAX1CNT = 0xFFFF;
            POS1CNT = 0;
            break;
        case MOTOR_ONE:
            //QEI2CONbits.CNTERR= 0; // No position count error has occurred
            QEI2CONbits.QEISIDL = 1; // Discontinue module operation when device enters Idle mode
            QEI2CONbits.QEIM = 7; // Quadrature Encoder Interface enabled (x4 mode) with position counter reset by match (MAXxCNT)
            QEI2CONbits.SWPAB = (get_motor_parameters(MOTOR_ONE).rotation >= 1) ? 1 : 0; // Phase A and Phase B inputs swapped
            QEI2CONbits.PCDOUT = 0; // Position counter direction status output disabled (Normal I/O pin operation)
            //QEI2CONbits.TQGATE= 0  // Timer gated time accumulation disabled
            //QEI2CONbits.TQCKPS = 0b00	// 1:1 prescale value
            QEI2CONbits.POSRES = 0; // Index pulse does not reset position counter

            DFLT2CONbits.QEOUT = 1; // Digital filter outputs enabled on QEAx/QEBx/INDXx pins
            DFLT2CONbits.QECK = 6; // 1:128 Clock divide for QEAx/QEBx/INDXx

            MAX2CNT = 0xFFFF;
            POS2CNT = 0;
            break;
    }
}

void InitIC(short motIdx) {
    switch (motIdx) {
        case MOTOR_ZERO:
            // Initialize Capture Module
            IC1CONbits.ICM = IC_DISABLE; // Disable Input Capture 1 module
            IC1CONbits.ICTMR = 1; // Select Timer2 as the IC1 Time base
            IC1CONbits.ICI = 0b01; // Interrupt on every second capture event
            IC1CONbits.ICM = IcMode[0]; // Generate capture event on every Rising edge

            // Enable Capture Interrupt And Timer2
            IPC0bits.IC1IP = INPUT_CAPTURE_LEVEL; // Setup IC1 interrupt priority level
            IFS0bits.IC1IF = 0; // Clear IC1 Interrupt Status Flag
            IEC0bits.IC1IE = 1; // Enable IC1 interrupt
            break;
        case MOTOR_ONE:
            // Initialize Capture Module
            IC2CONbits.ICM = IC_DISABLE; // Disable Input Capture 2 module
            IC2CONbits.ICTMR = 1; // Select Timer2 as the IC1 Time base
            IC2CONbits.ICI = 0b01; // Interrupt on every second capture event
            IC2CONbits.ICM = IcMode[0]; // Generate capture event on every Rising edge

            // Enable Capture Interrupt And Timer2
            IPC1bits.IC2IP = INPUT_CAPTURE_LEVEL; // Setup IC2 interrupt priority level
            IFS0bits.IC2IF = 0; // Clear IC2 Interrupt Status Flag
            IEC0bits.IC2IE = 1; // Enable IC2 interrupt
            break;
    }
}

void InitTimer2(void) {
    //T2CON = 10100000 00000000
    T2CONbits.TON = 0; // Disable Timer
    T2CONbits.TSIDL = 1; // Stop in Idle Mode bit
    T2CONbits.TGATE = 0; // Disable Gated Timer mode
    T2CONbits.TCKPS = 0b00; // Select 1:1 Prescaler
    T2CONbits.TCS = 0; // Select internal clock source
    TMR2 = 0x00; // Clear timer register
    PR2 = TMR2_VALUE; // Load the period value

    IPC1bits.T2IP = PWM_TIMER_LEVEL; // Set Timer 1 Interrupt Priority Level
    IFS0bits.T2IF = 0; // Clear Timer 1 Interrupt Flag
    IEC0bits.T2IE = 1; // Enable Timer1 interrupt

    T2CONbits.TON = 1; // Start Timer
}

void Motor_Init() {
    InitPWM();      ///< Open PWM
    int i;
#ifdef UNAV_V1
    /// ENABLE 1
    GPIO_INIT_TYPE(enable[0], A, 7, GPIO_OUTPUT);
    /// ENABLE 2
    GPIO_INIT_TYPE(enable[1], A, 10, GPIO_OUTPUT);
    // Encoders
    _TRISB10 = 1;
    _TRISB11 = 1;
    _TRISB6 = 1;
    _TRISB5 = 1;
    _TRISB12 = 0; // PWM1 +
    _TRISB12 = 0; // PWM1 -
    _TRISB12 = 0; // PWM2 +
    _TRISB12 = 0; // PWM2 -
#elif ROBOCONTROLLER_V3
    /// ENABLE 1
    GPIO_INIT_TYPE(enable[0], A, 1, GPIO_OUTPUT);
    /// ENABLE 2
    GPIO_INIT_TYPE(enable[1], A, 4, GPIO_OUTPUT);
    // Encodes
    _TRISC6 = 1; // QEA_1
    _TRISC7 = 1; // QEB_1
    _TRISC8 = 1; // QEA_2
    _TRISC9 = 1; // QEB_2
#elif MOTION_CONTROL
    /// ENABLE 1
    GPIO_INIT_TYPE(enable[0], B, 2, GPIO_OUTPUT);
    /// ENABLE 2
    GPIO_INIT_TYPE(enable[0], B, 3, GPIO_OUTPUT);
#endif
    for (i = 0; i < NUM_MOTORS; ++i) {
        InitQEI(i);                                             ///< Open QEI
        InitIC(i);                                              ///< Open Input Capture
        init_motor(i, &enable[i]);                              ///< Initialize variables for motors
        update_motor_parameters(i, init_motor_parameters());    ///< Initialize parameters for motors
        update_motor_pid(i, init_motor_pid());                  ///< Initialize PID controllers
        update_motor_emergency(i, init_motor_emergency());      ///< Initialize emergency procedure to stop
        update_motor_constraints(i, init_motor_constraints());  ///< Initialize constraints motor
        set_motor_state(i, STATE_CONTROL_DISABLE);              ///< Initialize state controller
    }
}

void SwitchIcPrescaler(int mode, int motIdx) {
    // here is the assignment of the ICx module to the correct wheel
    if (motIdx == 0) {
        IC1CONbits.ICM = IC_DISABLE; // turn off prescaler
        IC1CONbits.ICM = IcMode[mode];
        _IC1IF = 0; // interrupt flag reset
    } else {
        IC2CONbits.ICM = IC_DISABLE; // turn off prescaler
        IC2CONbits.ICM = IcMode[mode];
        _IC2IF = 0; // interrupt flag reset
    }
}

