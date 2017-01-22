/*
 * Copyright (C) 2014-2017 Officine Robotiche
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

#include <xc.h>
#include <pwm12.h>

#include "motor/init.h"

#define TMR2_VALUE 0xFFFF       // Timer2 - Value for overflow

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

// Dynamic Interrupt Capture
const ICMode_t ICMode[] = {
    {0b001, 1}, // 2X mode (default)
    {0b011, 2}, // 1X mode
    {0b100, 8}, // 1/4X mode
    {0b101, 32} // 1/16X mode
};
#define IC_DISABLE  0b000
#define ICMODE_DEFAULT 0

/**
 * xc16 PID source in: folder_install_microchip_software/xc16/1.2x/src/libdsp.zip
 * on zip file: asm/pid.s
 */
fractional abcCoefficient[MOTOR_SIZE][NUM_CONTROLLERS][3] __attribute__((section(".xbss, bss, xmemory")));
fractional controlHistory[MOTOR_SIZE][NUM_CONTROLLERS][3] __attribute__((section(".ybss, bss, ymemory")));

MOTOR_t motor[] = {
    MOTOR_INIT(0, ICMode, MOTOR_QEI_INIT(QEI1CON, POS1CNT)),
    MOTOR_INIT(1, ICMode, MOTOR_QEI_INIT(QEI2CON, POS2CNT))
};

/******************************************************************************/
/* Functions                                                                  */
/******************************************************************************/

void Motor_init_PWM(void) {
    // Holds the value to be loaded into dutycycle register
    unsigned int period;
    // Holds the value to be loaded into special event compare register
    unsigned int sptime;
    // Holds PWM configuration value
    unsigned int config1;
    // Holds the value be loaded into PWMCON1 register
    unsigned int config2;
    // Holds the value to configuration the special event trigger postscale and duty cycle
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
    // duty cycle reg=1, duty cycle=50% (motore fermo in LAP mode , update disable=0
    SetDCMCPWM1(1, 2048, 0);
    SetDCMCPWM1(2, 2048, 0);

    ConfigIntMCPWM1(PWM1_INT_DIS);
}

void Motor_init_timer2(void) {
    //T2CON = 10100000 00000000
    T2CONbits.TON = 0; // Disable Timer
    T2CONbits.TSIDL = 1; // Stop in Idle Mode bit
    T2CONbits.TGATE = 0; // Disable Gated Timer mode
    T2CONbits.TCKPS = 0b00; // Select 1:1 Prescaler
    T2CONbits.TCS = 0; // Select internal clock source
    TMR2 = 0x00; // Clear timer register
    PR2 = TMR2_VALUE; // Load the period value

    IPC1bits.T2IP = 1; // Set Timer 1 Interrupt Priority Level
    IFS0bits.T2IF = 0; // Clear Timer 1 Interrupt Flag
    IEC0bits.T2IE = 1; // Enable Timer1 interrupt

    T2CONbits.TON = 1; // Start Timer
}

void Motor_init_IC(MOTOR_t *motor) {
    switch (motor->Idx) {
        case MOTOR_ZERO:
            // Initialize Capture Module
            IC1CONbits.ICM = IC_DISABLE; // Disable Input Capture 1 module
            IC1CONbits.ICTMR = 1; // Select Timer2 as the IC1 Time base
            IC1CONbits.ICI = 0b00; // Interrupt on every second capture event
            IC1CONbits.ICM = ICMode[ICMODE_DEFAULT].mode; // Generate capture event on every Rising edge

            // Enable Capture Interrupt And Timer2
            IPC0bits.IC1IP = 1; // Setup IC1 interrupt priority level
            IFS0bits.IC1IF = 0; // Clear IC1 Interrupt Status Flag
            IEC0bits.IC1IE = 1; // Enable IC1 interrupt
            break;
        case MOTOR_ONE:
            // Initialize Capture Module
            IC2CONbits.ICM = IC_DISABLE; // Disable Input Capture 2 module
            IC2CONbits.ICTMR = 1; // Select Timer2 as the IC1 Time base
            IC2CONbits.ICI = 0b00; // Interrupt on every second capture event
            IC2CONbits.ICM = ICMode[ICMODE_DEFAULT].mode; // Generate capture event on every Rising edge

            // Enable Capture Interrupt And Timer2
            IPC1bits.IC2IP = 1; // Setup IC2 interrupt priority level
            IFS0bits.IC2IF = 0; // Clear IC2 Interrupt Status Flag
            IEC0bits.IC2IE = 1; // Enable IC2 interrupt
            break;
    }
}

void Motor_init_QEI(MOTOR_t *motor) {
    switch (motor->Idx) {
        case MOTOR_ZERO:
            //QEI1CONbits.CNTERR= 0; // No position count error has occurred
            QEI1CONbits.QEISIDL = 1; // Discontinue module operation when device enters Idle mode
            QEI1CONbits.QEIM = 7; // Quadrature Encoder Interface enabled (x4 mode) with position counter reset by match (MAXxCNT)
//            QEI1CONbits.SWPAB = (Motor_get_parameters(motor).rotation >= 1) ? 1 : 0; // Phase A and Phase B inputs swapped
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
//            QEI2CONbits.SWPAB = (Motor_get_parameters(motor).rotation >= 1) ? 1 : 0; // Phase A and Phase B inputs swapped
            QEI2CONbits.PCDOUT = 0; // Position counter direction status output disabled (Normal I/O pin operation)
            //QEI2CONbits.TQGATE= 0  // Timer gated time accumulation disabled
            //QEI2CONbits.TQCKPS = 0b00	// 1:1 pre scale value
            QEI2CONbits.POSRES = 0; // Index pulse does not reset position counter

            DFLT2CONbits.QEOUT = 1; // Digital filter outputs enabled on QEAx/QEBx/INDXx pins
            DFLT2CONbits.QECK = 6; // 1:128 Clock divide for QEAx/QEBx/INDXx

            MAX2CNT = 0xFFFF;
            POS2CNT = 0;
            break;
    }
}

void Motor_start(void) {
    unsigned int i;
    
    // Initialization PWM
    Motor_init_PWM();
    // Open Timer2 for InputCapture 1 & 2
    Motor_init_timer2();
    // Initialization motors
    for(i = 0; i < MOTOR_SIZE; ++i) {
        // Initialization QEI
        Motor_init_QEI(&motor[i]);
        // Open Input Capture
        Motor_init_IC(&motor[i]);
        // Initialization Motor controller
        Motor_Init(&motor[i], &abcCoefficient[i][0][0], &controlHistory[i][0][0]);
    }
}

void __attribute__((interrupt, auto_psv)) _IC1Interrupt(void) {
    int dir = (QEI1CONbits.UPDN ? 1 : -1);
    // Run the Input Capture controller
    Motor_IC_controller(&motor[MOTOR_ZERO], IC1BUF, dir);
    // Clear the interrupt
    IFS0bits.IC1IF = 0;
    // ISR exit
    taskYIELD();
}

void __attribute__((interrupt, auto_psv)) _IC2Interrupt(void) {
    // Run the Input Capture controller
    int dir = (QEI2CONbits.UPDN ? 1 : -1);
    Motor_IC_controller(&motor[MOTOR_ONE], IC2BUF, dir);
    // Clear the interrupt
    IFS0bits.IC2IF = 0;
    // ISR exit
    taskYIELD();
}

void __attribute__((interrupt, auto_psv)) _T2Interrupt(void) {
    // Increase timer value
    Motor_IC_timer(&motor[MOTOR_ZERO]);
    Motor_IC_timer(&motor[MOTOR_ONE]);
    IFS0bits.T2IF = 0; // interrupt flag reset
    // ISR exit
    taskYIELD();
}