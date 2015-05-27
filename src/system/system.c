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
#include "packet/frame_motion.h"
#include "communication/serial.h"
#include "motors/motor_control.h"
#include "high_control/manager.h"

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

unsigned char version_code[] = "v0.5";
unsigned char type_board[] = "Motor Control";
system_parameter_t parameter_system;

extern unsigned char BufferTx[MAX_BUFF_TX] __attribute__((space(dma)));

// ADC buffer, 2 channels (AN0, AN1), 32 bytes each, 2 x 32 = 64 bytes
extern int AdcBuffer[ADC_CHANNELS][ADC_BUFF] __attribute__((space(dma), aligned(256)));

process_t default_process[NUM_PROCESS_DEFAULT];
process_t motor_process[PROCESS_MOTOR_LENGTH];
process_t motion_process[PROCESS_MOTION_LENGTH];

// From motors PID
//extern parameter_motor_t parameter_motor_left, parameter_motor_right;

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
    strcpy(motor_process[LEFT_PROCESS_PID].name, LEFT_PROCESS_PID_STRING);
    motor_process[LEFT_PROCESS_PID].time = 0;
    motor_process[LEFT_PROCESS_PID].priority = VEL_PID_LEVEL;
    motor_process[LEFT_PROCESS_PID].frequency = 1;
    //Init right process pid
    strcpy(motor_process[RIGHT_PROCESS_PID].name, RIGHT_PROCESS_PID_STRING);
    motor_process[RIGHT_PROCESS_PID].time = 0;
    motor_process[RIGHT_PROCESS_PID].priority = VEL_PID_LEVEL;
    motor_process[RIGHT_PROCESS_PID].frequency = 1;
    //Init left process measure
    strcpy(motor_process[LEFT_PROCESS_MEASURE].name, LEFT_PROCESS_MEASURE_STRING);
    motor_process[LEFT_PROCESS_MEASURE].time = 0;
    motor_process[LEFT_PROCESS_MEASURE].priority = MEASURE_LEVEL;
    motor_process[LEFT_PROCESS_MEASURE].frequency = 1;
    //Init right process measure
    strcpy(motor_process[RIGHT_PROCESS_MEASURE].name, RIGHT_PROCESS_MEASURE_STRING);
    motor_process[RIGHT_PROCESS_MEASURE].time = 0;
    motor_process[RIGHT_PROCESS_MEASURE].priority = MEASURE_LEVEL;
    motor_process[RIGHT_PROCESS_MEASURE].frequency = 1;
    
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

void set_process(uint8_t command, system_task_t process_state) {
    if (process_state.hashmap == HASHMAP_SYSTEM) {
        switch (command) {
            case SYSTEM_TASK_TIME:
                default_process[process_state.number].time = process_state.data;
                break;
            case SYSTEM_TASK_PRIORITY:
                default_process[process_state.number].priority = process_state.data;
                break;
            case SYSTEM_TASK_FRQ:
                default_process[process_state.number].frequency = process_state.data;
                break;
        }
    } else if (process_state.hashmap == HASHMAP_MOTOR) {
        switch (command) {
            case SYSTEM_TASK_TIME:
                motor_process[process_state.number].time = process_state.data;
                break;
            case SYSTEM_TASK_PRIORITY:
                motor_process[process_state.number].priority = process_state.data;
                break;
            case SYSTEM_TASK_FRQ:
                motor_process[process_state.number].frequency = process_state.data;
                break;
        }
    } else if (process_state.hashmap == HASHMAP_MOTION) {
        switch (command) {
            case SYSTEM_TASK_TIME:
                motion_process[process_state.number].time = process_state.data;
                break;
            case SYSTEM_TASK_PRIORITY:
                motion_process[process_state.number].priority = process_state.data;
                break;
            case SYSTEM_TASK_FRQ:
                motion_process[process_state.number].frequency = process_state.data;
                break;
        }
    }
}

system_task_t get_process(uint8_t command, system_task_t process_state) {
    if (process_state.hashmap == HASHMAP_SYSTEM) {
        switch (command) {
            case SYSTEM_TASK_TIME:
                process_state.data = default_process[process_state.number].time;
                break;
            case SYSTEM_TASK_PRIORITY:
                process_state.data = default_process[process_state.number].priority;
                break;
            case SYSTEM_TASK_FRQ:
                process_state.data = default_process[process_state.number].frequency;
                break;
            case SYSTEM_TASK_NUM:
                process_state.data = NUM_PROCESS_DEFAULT;
                break;
        }
    } else if (process_state.hashmap == HASHMAP_MOTOR) {
        switch (command) {
            case SYSTEM_TASK_TIME:
                process_state.data = motor_process[process_state.number].time;
                break;
            case SYSTEM_TASK_PRIORITY:
                process_state.data = motor_process[process_state.number].priority;
                break;
            case SYSTEM_TASK_FRQ:
                process_state.data = motor_process[process_state.number].frequency;
                break;
            case SYSTEM_TASK_NUM:
                process_state.data = PROCESS_MOTOR_LENGTH;
                break;
        }
    } else if (process_state.hashmap == HASHMAP_MOTION) {
        switch (command) {
            case SYSTEM_TASK_TIME:
                process_state.data = motion_process[process_state.number].time;
                break;
            case SYSTEM_TASK_PRIORITY:
                process_state.data = motion_process[process_state.number].priority;
                break;
            case SYSTEM_TASK_FRQ:
                process_state.data = motion_process[process_state.number].frequency;
                break;
            case SYSTEM_TASK_NUM:
                process_state.data = PROCESS_MOTION_LENGTH;
                break;
        }
    }
    return process_state;
}

system_task_name_t get_process_name(system_task_name_t process_name) {
    if (process_name.hashmap == HASHMAP_SYSTEM) {
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
    return PACKET_ACK;
}

unsigned char update_frequency(void) {
    if (motor_process[LEFT_PROCESS_PID].frequency == 0 || motor_process[RIGHT_PROCESS_PID].frequency == 0) {
        VEL_PID_ENABLE = 0; // Disable Output Compare Channel 1 interrupt
    } else
        VEL_PID_ENABLE = 1; // Enable Output Compare Channel 1 interrupt
    if (motion_process[PROCESS_ODOMETRY].frequency == 0 || motion_process[PROCESS_VELOCITY].frequency == 0) {
        DEAD_RECK_ENABLE = 0; // Disable RTC interrupt
    } else
        DEAD_RECK_ENABLE = 1; // Enable RTC interrupt
    return PACKET_ACK;
}

system_service_t services(system_service_t service) {
    system_service_t service_send;
    service_send.command = service.command;
    switch (service.command) {
        case SERVICE_CODE_DATE:
            memcpy(service_send.buffer, version_date_, sizeof (version_date_));
            service_send.buffer[sizeof (version_date_) - 1] = ' ';
            memcpy(service_send.buffer + sizeof (version_date_), version_time_, sizeof (version_time_));
            break;
        case SERVICE_CODE_BOARD_NAME:
            memcpy(service_send.buffer, name_board, sizeof (name_board));
            break;
        case SERVICE_CODE_BOARD_TYPE:
            memcpy(service_send.buffer, type_board, sizeof (type_board));
            break;
        case SERVICE_CODE_VERSION:
            memcpy(service_send.buffer, version_code, sizeof (version_code));
            break;
        case SERVICE_CODE_AUTHOR:
            memcpy(service_send.buffer, author_code, sizeof (author_code));
            break;
        case SERVICE_RESET:
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
    VEL_PID_PRIORITY = motor_process[LEFT_PROCESS_PID].priority; // Set Output Compare Channel 1 Priority Level
    IFS0bits.OC1IF = 0; // Clear Output Compare Channel 1 Interrupt Flag
    VEL_PID_ENABLE = 1; // Enable Output Compare Channel 1 interrupt

    //For Parsing UART message
    RX_PARSER_ENABLE = 0; // Disable Output Compare Channel 2 interrupt
    RX_PARSER_PRIORITY = RX_PARSER_LEVEL; // Set Output Compare Channel 2 Priority Level
    IFS0bits.OC2IF = 0; // Clear Output Compare Channel 2 Interrupt Flag
    RX_PARSER_ENABLE = 1; // Enable Output Compare Channel 2 interrupt

    //For measure position and velocity estimation
    MEASURE_ENABLE = 0; // Disable Output Compare Channel 3 interrupt
    MEASURE_PRIORITY = motor_process[LEFT_PROCESS_MEASURE].priority; // Set Output Compare Channel 3 Priority Level
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

    DMA1CNT = MAX_BUFF_TX - 1; // 32 DMA requests
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
