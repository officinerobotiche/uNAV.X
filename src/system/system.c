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
#include <pwm12.h>
#include <string.h>

#include "system/user.h"
#include "system/system.h"   /* variables/params used by system.c             */
#include "packet/packet.h"
#include "packet/motion.h"
#include "communication/serial.h"
#include "control/motors.h"

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

unsigned int reset_count = 0;
unsigned char version_date_[] = __DATE__;
unsigned char version_time_[] = __TIME__;
#ifdef UNAV_V1
unsigned char name_board[] = "uNAV";
#elif ROBOCONTROLLER_V3
unsigned char name_board[] = "RoboController";
#elif MOTION_CONTROL
unsigned char name_board[] = "Motion Control";
#endif
#ifdef MOTION_CONTROL
unsigned char author_code[] = "Raffaello Bonghi";
#else
unsigned char author_code[] = "Officine Robotiche";
#endif

unsigned char version_code[] = "v0.4";
unsigned char type_board[] = "Motor Control";
parameter_system_t parameter_system;

extern unsigned char BufferTx[MAX_TX_BUFF] __attribute__((space(dma)));

// ADC buffer, 2 channels (AN0, AN1), 32 bytes each, 2 x 32 = 64 bytes
extern int AdcBuffer[ADC_CHANNELS][ADC_BUFF] __attribute__((space(dma), aligned(256)));

process_t default_process[2];
process_t motor_process[PROCESS_MOTOR_LENGTH];
process_t motion_process[PROCESS_MOTION_LENGTH];

// From motors PID
//extern parameter_motor_t parameter_motor_left, parameter_motor_right;

// Dynamic Interrupt Capture
#define IC_MODE0    0b001 // 2X mode (default)
#define IC_MODE1    0b011 // 1X mode
#define IC_MODE2    0b100 // 1/4X mode
#define IC_MODE3    0b101 // 1/16X mode
#define IC_DISABLE  0b00
const int IcMode[4] = {IC_MODE0, IC_MODE1, IC_MODE2, IC_MODE3};

/******************************************************************************/
/* System Level Functions                                                     */
/*                                                                            */
/* Custom oscillator configuration funtions, reset source evaluation          */
/* functions, and other non-peripheral microcontroller initialization         */
/* functions get placed in system.c.                                          */
/*                                                                            */
/******************************************************************************/

/* Refer to the device Family Reference Manual Oscillator section for
information about available oscillator configurations.  Typically
this would involve configuring the oscillator tuning register or clock
switching useing the compiler's __builtin_write_OSCCON functions.
Refer to the C Compiler for PIC24 MCUs and dsPIC DSCs User Guide in the
compiler installation directory /doc folder for documentation on the
__builtin functions.*/

void init_process(void) {
    strcpy(default_process[PROCESS_IDLE].name, "idle");
    default_process[PROCESS_IDLE].time = 0;
    strcpy(default_process[PROCESS_PARSE].name, "parse");
    default_process[PROCESS_PARSE].time = 0;
    //Init left process pid
    strcpy(motor_process[PROCESS_PID_LEFT].name, PID_LEFT_STRING);
    motor_process[PROCESS_PID_LEFT].time = 0;
    motor_process[PROCESS_PID_LEFT].priority = VEL_PID_LEVEL;
    motor_process[PROCESS_PID_LEFT].frequency = 1;
    //Init right process pid
    strcpy(motor_process[PROCESS_PID_RIGHT].name, PID_RIGHT_STRING);
    motor_process[PROCESS_PID_RIGHT].time = 0;
    motor_process[PROCESS_PID_RIGHT].priority = VEL_PID_LEVEL;
    motor_process[PROCESS_PID_RIGHT].frequency = 1;
    //Init left process measure
    strcpy(motor_process[PROCESS_MEASURE_VEL_LEFT].name, MEASURE_VEL_LEFT_STRING);
    motor_process[PROCESS_MEASURE_VEL_LEFT].time = 0;
    motor_process[PROCESS_MEASURE_VEL_LEFT].priority = MEASURE_LEVEL;
    motor_process[PROCESS_MEASURE_VEL_LEFT].frequency = 1;
    //Init right process measure
    strcpy(motor_process[PROCESS_MEASURE_VEL_RIGHT].name, MEASURE_VEL_RIGHT_STRING);
    motor_process[PROCESS_MEASURE_VEL_RIGHT].time = 0;
    motor_process[PROCESS_MEASURE_VEL_RIGHT].priority = MEASURE_LEVEL;
    motor_process[PROCESS_MEASURE_VEL_RIGHT].frequency = 1;
    
    //Init process odometry
    strcpy(motion_process[PROCESS_ODOMETRY].name, ODOMETRY_STRING);
    motion_process[PROCESS_ODOMETRY].time = 0;
    motion_process[PROCESS_ODOMETRY].priority = DEAD_RECK_LEVEL;
    motion_process[PROCESS_ODOMETRY].frequency = 10;
    //Init process odometry
    strcpy(motion_process[PROCESS_VELOCITY].name, VELOCITY_STRING);
    motion_process[PROCESS_VELOCITY].time = 0;
    motion_process[PROCESS_VELOCITY].priority = VEL_PID_LEVEL;
    motion_process[PROCESS_VELOCITY].frequency = 10;

    parameter_system.step_timer = (int) (TMR1_VALUE);
    parameter_system.int_tm_mill = (int) (TCTMR1 * 1000);
}

void set_process(uint8_t command, process_state_t process_state) {
    if (process_state.hashmap == HASHMAP_DEFAULT) {
        switch (command) {
            case PROCESS_TIME:
                default_process[process_state.number].time = process_state.data;
                break;
            case PROCESS_PRIORITY:
                default_process[process_state.number].priority = process_state.data;
                break;
            case PROCESS_FRQ:
                default_process[process_state.number].frequency = process_state.data;
                break;
        }
    } else if (process_state.hashmap == HASHMAP_MOTOR) {
        switch (command) {
            case PROCESS_TIME:
                motor_process[process_state.number].time = process_state.data;
                break;
            case PROCESS_PRIORITY:
                motor_process[process_state.number].priority = process_state.data;
                break;
            case PROCESS_FRQ:
                motor_process[process_state.number].frequency = process_state.data;
                break;
        }
    } else if (process_state.hashmap == HASHMAP_MOTION) {
        switch (command) {
            case PROCESS_TIME:
                motion_process[process_state.number].time = process_state.data;
                break;
            case PROCESS_PRIORITY:
                motion_process[process_state.number].priority = process_state.data;
                break;
            case PROCESS_FRQ:
                motion_process[process_state.number].frequency = process_state.data;
                break;
        }
    }
}

process_state_t get_process(uint8_t command, process_state_t process_state) {
    if (process_state.hashmap == HASHMAP_DEFAULT) {
        switch (command) {
            case PROCESS_TIME:
                process_state.data = default_process[process_state.number].time;
                break;
            case PROCESS_PRIORITY:
                process_state.data = default_process[process_state.number].priority;
                break;
            case PROCESS_FRQ:
                process_state.data = default_process[process_state.number].frequency;
                break;
        }
    } else if (process_state.hashmap == HASHMAP_MOTOR) {
        switch (command) {
            case PROCESS_TIME:
                process_state.data = motor_process[process_state.number].time;
                break;
            case PROCESS_PRIORITY:
                process_state.data = motor_process[process_state.number].priority;
                break;
            case PROCESS_FRQ:
                process_state.data = motor_process[process_state.number].frequency;
                break;
        }
    } else if (process_state.hashmap == HASHMAP_MOTION) {
        switch (command) {
            case PROCESS_TIME:
                process_state.data = motion_process[process_state.number].time;
                break;
            case PROCESS_PRIORITY:
                process_state.data = motion_process[process_state.number].priority;
                break;
            case PROCESS_FRQ:
                process_state.data = motion_process[process_state.number].frequency;
                break;
        }
    }
    return process_state;
}

process_name_t get_process_name(process_name_t process_name) {
    if (process_name.hashmap == HASHMAP_DEFAULT) {
        strcpy(process_name.data, default_process[process_name.number].name);
    } else if (process_name.hashmap == HASHMAP_MOTOR) {
        strcpy(process_name.data, motor_process[process_name.number].name);
    } else if (process_name.hashmap == HASHMAP_MOTION) {
        strcpy(process_name.data, motion_process[process_name.number].name);
    }
    return process_name;
}

unsigned char update_priority(void) {
    default_process[PROCESS_IDLE].time = 0;
    InitInterrupts();
    return ACK;
}

unsigned char update_frequency(void) {
    if (motor_process[PROCESS_PID_LEFT].frequency == 0 || motor_process[PROCESS_PID_RIGHT].frequency == 0) {
        VEL_PID_ENABLE = 0; // Disable Output Compare Channel 1 interrupt
    } else
        VEL_PID_ENABLE = 1; // Enable Output Compare Channel 1 interrupt
    if (motion_process[PROCESS_ODOMETRY].frequency == 0 || motion_process[PROCESS_VELOCITY].frequency == 0) {
        DEAD_RECK_ENABLE = 0; // Disable RTC interrupt
    } else
        DEAD_RECK_ENABLE = 1; // Enable RTC interrupt
    return ACK;
}

services_t services(services_t service) {
    services_t service_send;
    service_send.command = service.command;
    switch (service.command) {
        case DATE_CODE:
            memcpy(service_send.buffer, version_date_, sizeof (version_date_));
            service_send.buffer[sizeof (version_date_) - 1] = ' ';
            memcpy(service_send.buffer + sizeof (version_date_), version_time_, sizeof (version_time_));
            break;
        case NAME_BOARD:
            memcpy(service_send.buffer, name_board, sizeof (name_board));
            break;
        case TYPE_BOARD:
            memcpy(service_send.buffer, type_board, sizeof (type_board));
            break;
        case VERSION_CODE:
            memcpy(service_send.buffer, version_code, sizeof (version_code));
            break;
        case AUTHOR_CODE:
            memcpy(service_send.buffer, author_code, sizeof (author_code));
            break;
        case RESET:
            if (reset_count < 3) {
                reset_count++;
            } else {
                SET_CPU_IPL(7); // disable all user interrupts
                //DelayN1ms(200);
                asm("RESET");
            }
            break;
        default:
            break;
    }
    return service_send;
}

void InitInterrupts(void) {
    //For PID velocity control
    VEL_PID_ENABLE = 0; // Disable Output Compare Channel 1 interrupt
    VEL_PID_PRIORITY = motor_process[PROCESS_PID_LEFT].priority; // Set Output Compare Channel 1 Priority Level
    IFS0bits.OC1IF = 0; // Clear Output Compare Channel 1 Interrupt Flag
    VEL_PID_ENABLE = 1; // Enable Output Compare Channel 1 interrupt

    //For Parsing UART message
    RX_PARSER_ENABLE = 0; // Disable Output Compare Channel 2 interrupt
    RX_PARSER_PRIORITY = RX_PARSER_LEVEL; // Set Output Compare Channel 2 Priority Level
    IFS0bits.OC2IF = 0; // Clear Output Compare Channel 2 Interrupt Flag
    RX_PARSER_ENABLE = 1; // Enable Output Compare Channel 2 interrupt

    //For measure position and velocity estimation
    MEASURE_ENABLE = 0; // Disable Output Compare Channel 3 interrupt
    MEASURE_PRIORITY = motor_process[PROCESS_MEASURE_VEL_LEFT].priority; // Set Output Compare Channel 3 Priority Level
    IFS1bits.OC3IF = 0; // Clear Output Compare Channel 3 Interrupt Flag
    MEASURE_ENABLE = 1; // Enable Output Compare Channel 3 interrupt
    
    // For dead reckoning
    DEAD_RECK_ENABLE = 0; // Disable RTC interrupt
    DEAD_RECK_PRIORITY = motion_process[PROCESS_ODOMETRY].priority; // Set RTC Priority Level
    IFS3bits.RTCIF = 0; // Clear RTC Interrupt Flag
    DEAD_RECK_ENABLE = 1; // Enable RTC interrupt
}

void ConfigureOscillator(void) {
    PLLFBD = 30; // M=32  //Old configuration: PLLFBD=29 - M=31
    CLKDIVbits.PLLPOST = 0; // N1=2
    CLKDIVbits.PLLPRE = 0; // N2=2
    // Disable Watch Dog Timer
    RCONbits.SWDTEN = 0;
    // Clock switching to incorporate PLL
    // Initiate Clock Switch to Primary
    __builtin_write_OSCCONH(0x03); // Oscillator with PLL (NOSC=0b011)
    __builtin_write_OSCCONL(0x01); // Start clock switching
    while (OSCCONbits.COSC != 0b011); // Wait for Clock switch to occur
    while (OSCCONbits.LOCK != 1) {
    }; // Wait for PLL to lock
}

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
    config1 = PWM1_EN & PWM1_IDLE_CON & PWM1_OP_SCALE1 & PWM1_IPCLK_SCALE1 &
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

void InitQEI1(void) {
    //QEI1CONbits.CNTERR= 0; // No position count error has occurred
    QEI1CONbits.QEISIDL = 1; // Discontinue module operation when device enters Idle mode
    QEI1CONbits.QEIM = 7; // Quadrature Encoder Interface enabled (x4 mode) with position counter reset by match (MAXxCNT)
    QEI1CONbits.SWPAB = (get_parameter_motor(MOTOR_ZERO).versus >= 1) ? 1 : 0; // Phase A and Phase B inputs swapped
    QEI1CONbits.PCDOUT = 0; // Position counter direction status output disabled (Normal I/O pin operation)
    //QEI1CONbits.TQGATE= 0  // Timer gated time accumulation disabled
    //QEI1CONbits.TQCKPS = 0b00	// 1:1 prescale value
    QEI1CONbits.POSRES = 0; // Index pulse does not reset position counter

    DFLT1CONbits.QEOUT = 1; // Digital filter outputs enabled on QEAx/QEBx/INDXx pins
    DFLT1CONbits.QECK = 6; // 1:128 Clock divide for QEAx/QEBx/INDXx

    MAX1CNT = 0xFFFF;
    POS1CNT = 0;
}

void InitQEI2(void) {
    //QEI2CONbits.CNTERR= 0; // No position count error has occurred
    QEI2CONbits.QEISIDL = 1; // Discontinue module operation when device enters Idle mode
    QEI2CONbits.QEIM = 7; // Quadrature Encoder Interface enabled (x4 mode) with position counter reset by match (MAXxCNT)
    QEI2CONbits.SWPAB = (get_parameter_motor(MOTOR_ONE).versus >= 1) ? 1 : 0; // Phase A and Phase B inputs swapped
    QEI2CONbits.PCDOUT = 0; // Position counter direction status output disabled (Normal I/O pin operation)
    //QEI2CONbits.TQGATE= 0  // Timer gated time accumulation disabled
    //QEI2CONbits.TQCKPS = 0b00	// 1:1 prescale value
    QEI2CONbits.POSRES = 0; // Index pulse does not reset position counter

    DFLT2CONbits.QEOUT = 1; // Digital filter outputs enabled on QEAx/QEBx/INDXx pins
    DFLT2CONbits.QECK = 6; // 1:128 Clock divide for QEAx/QEBx/INDXx

    MAX2CNT = 0xFFFF;
    POS2CNT = 0;
}

void InitIC1(void) {
    // Initialize Capture Module
    IC1CONbits.ICM = IC_DISABLE; // Disable Input Capture 1 module
    IC1CONbits.ICTMR = 1; // Select Timer2 as the IC1 Time base
    IC1CONbits.ICI = 0b01; // Interrupt on every second capture event
    IC1CONbits.ICM = IcMode[0]; // Generate capture event on every Rising edge

    // Enable Capture Interrupt And Timer2
    IPC0bits.IC1IP = INPUT_CAPTURE_LEVEL; // Setup IC1 interrupt priority level
    IFS0bits.IC1IF = 0; // Clear IC1 Interrupt Status Flag
    IEC0bits.IC1IE = 1; // Enable IC1 interrupt
}

void InitIC2(void) {
    // Initialize Capture Module
    IC2CONbits.ICM = IC_DISABLE; // Disable Input Capture 2 module
    IC2CONbits.ICTMR = 1; // Select Timer2 as the IC1 Time base
    IC2CONbits.ICI = 0b01; // Interrupt on every second capture event
    IC2CONbits.ICM = IcMode[0]; // Generate capture event on every Rising edge

    // Enable Capture Interrupt And Timer2
    IPC1bits.IC2IP = INPUT_CAPTURE_LEVEL; // Setup IC2 interrupt priority level
    IFS0bits.IC2IF = 0; // Clear IC2 Interrupt Status Flag
    IEC0bits.IC2IE = 1; // Enable IC2 interrupt
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

void InitTimer1(void) {
    //T1CON = 10100000 00000000
    T1CONbits.TON = 0; // Disable Timer
    T1CONbits.TSIDL = 1; // Stop in Idle Mode bit
    T1CONbits.TGATE = 0; // Disable Gated Timer mode
    T1CONbits.TCKPS = 0b00; // Select 1:1 Prescaler
    T1CONbits.TSYNC = 0; // Disable Synchronization
    T1CONbits.TCS = 0; // Select internal clock source
    TMR1 = 0x00; // Clear timer register
    PR1 = TMR1_VALUE; // Load the period value

    IPC0bits.T1IP = SYS_TIMER_LEVEL; // Set Timer 1 Interrupt Priority Level
    IFS0bits.T1IF = 0; // Clear Timer 1 Interrupt Flag
    IEC0bits.T1IE = 1; // Enable Timer1 interrupt

    T1CONbits.TON = 1; // Start Timer
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

void InitUART1(void) {
    U1MODEbits.STSEL = 0; // 1-stop bit
    U1MODEbits.PDSEL = 0; // No Parity, 8-data bits
    U1MODEbits.ABAUD = 0; // Auto-Baud Disabled
    U1MODEbits.BRGH = 0; // Low Speed mode

    U1BRG = BRGVAL; // BAUD Rate Setting on System.h

    U1STAbits.UTXISEL0 = 0; // Interrupt after one Tx character is transmitted
    U1STAbits.UTXISEL1 = 0;

    IEC0bits.U1TXIE = 0; // Disable UART Tx interrupt
    U1STAbits.URXISEL = 0; // Interrupt after one RX character is received

    U1MODEbits.UARTEN = 1; // Enable UART
    U1STAbits.UTXEN = 1; // Enable UART Tx

    IEC4bits.U1EIE = 0;
    IPC2bits.U1RXIP = UART_RX_LEVEL; // Set UART Rx Interrupt Priority Level
    IFS0bits.U1RXIF = 0; // Reset RX interrupt flag
    IEC0bits.U1RXIE = 1; // Enable RX interrupt
}

void InitDMA0(void) {
    DMA0CNT = TOT_ADC_BUFF - 1; // 64 DMA request
    DMA0REQ = 13; // Select ADC1 as DMA Request source

    DMA0CONbits.AMODE = 2; // Peripheral Indirect Addressing mode
    DMA0CONbits.MODE = 0; // Continuous

    DMA0STA = __builtin_dmaoffset(AdcBuffer);
    DMA0PAD = (volatile unsigned int) &ADC1BUF0; // Point DMA to ADC1BUF0

    IFS0bits.DMA0IF = 0; // Clear DMA Interrupt Flag
    IPC1bits.DMA0IP = ADC_DMA_LEVEL; // Set DMA Interrupt Priority Level
    IEC0bits.DMA0IE = 1; // Enable DMA interrupt
    DMA0CONbits.CHEN = 1; // Enable DMA
}

void InitDMA1(void) {
    //DMA1CON = 0x2001;			// One-Shot, Post-Increment, RAM-to-Peripheral

    DMA1CONbits.CHEN = 0;
    DMA1CONbits.SIZE = 1;
    DMA1CONbits.DIR = 1;
    DMA1CONbits.HALF = 0;
    DMA1CONbits.NULLW = 0;
    DMA1CONbits.AMODE = 0;
    DMA1CONbits.MODE = 1;

    DMA1CNT = MAX_TX_BUFF - 1; // 32 DMA requests
    DMA1REQ = 0x000c; // Select UART1 Transmitter

    DMA1STA = __builtin_dmaoffset(BufferTx);
    DMA1PAD = (volatile unsigned int) &U1TXREG;

    IPC3bits.DMA1IP = UART_TX_LEVEL; // Set DMA Interrupt Priority Level
    IFS0bits.DMA1IF = 0; // Clear DMA Interrupt Flag
    IEC0bits.DMA1IE = 1; // Enable DMA interrupt
}

void InitADC(void) {
    AD1CON1bits.FORM = 0; // Data Output Format: Integer
    AD1CON1bits.SSRC = 3; // Sample Clock Source: Internal counter sampling and starts convertions (auto-convert)
    AD1CON1bits.ASAM = 1; // ADC Sample Control: Sampling begins immediately after conversion
    AD1CON1bits.AD12B = 0; // 10-bit ADC operation
    AD1CON1bits.ADSIDL = 1; // stop in idle
    AD1CON1bits.SIMSAM = 1; // CH0 CH1 sampled simultaneously

    AD1CON2bits.CSCNA = 0; // Input scan: Do not scan inputs
    AD1CON2bits.CHPS = 1; // Convert CH0 and CH1
    AD1CON2bits.BUFM = 0; // filling buffer from start address
    AD1CON2bits.ALTS = 0; // sample A

    AD1CON3bits.ADRC = 0; // ADC Clock is derived from Systems Clock
    AD1CON3bits.SAMC = 0b11111; // 31 Tad auto sample time
    AD1CON3bits.ADCS = ADC_BUFF - 1; // ADC Conversion Clock Tad=Tcy*(ADCS+1)= (1/40M)*64 = 1.6us (625Khz)
    // ADC Conversion Time for 10-bit Tc=12*Tab = 19.2us

    AD1CON1bits.ADDMABM = 0; // DMA buffers are built in scatter/gather mode
    AD1CON2bits.SMPI = 0b0001; // number of DMA buffers -1
    AD1CON4bits.DMABL = 0b110; // 64 word DMA buffer for each analog input

    AD1CHS123bits.CH123NB = 0; // don't care -> sample B
    AD1CHS123bits.CH123SB = 0; // don't care -> sample B
    AD1CHS123bits.CH123NA = 0; // CH1,2,3 negative input = Vrefl
    AD1CHS123bits.CH123SA = 0; // CH1 = AN0, CH2=AN1, CH3=AN2

    AD1CHS0bits.CH0NB = 0; // don't care -> sample B
    AD1CHS0bits.CH0SB = 0; // don't care -> sample B
    AD1CHS0bits.CH0NA = 0; // CH0 neg -> Vrefl
    AD1CHS0bits.CH0SA = 1; // CH0 pos -> AN1

    AD1PCFGL = 0xFFFF; // set all Analog ports as digital
    AD1PCFGLbits.PCFG0 = 0; // AN0
    AD1PCFGLbits.PCFG1 = 0; // AN1

    IFS0bits.AD1IF = 0; // Clear the A/D interrupt flag bit
    IEC0bits.AD1IE = 0; // Do Not Enable A/D interrupt
    AD1CON1bits.ADON = 1; // module on
}
