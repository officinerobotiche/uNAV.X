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

#ifndef SYSTEM_H
#define	SYSTEM_H

#ifdef	__cplusplus
extern "C" {
#endif

    #include "packet/packet.h"

/******************************************************************************/
/* System Level #define Macros                                                */
/******************************************************************************/
    
    /* Name interrupt */
    #define VEL_PID_PRIORITY IPC0bits.OC1IP
    #define RX_PARSER_PRIORITY IPC1bits.OC2IP
    #define DEAD_RECK_PRIORITY IPC15bits.RTCIP

    /* Interrupt enable */
    #define VEL_PID_ENABLE IEC0bits.OC1IE
    #define RX_PARSER_ENABLE IEC0bits.OC2IE
    #define DEAD_RECK_ENABLE IEC3bits.RTCIE

    /* Interrupt priority */
    /* Max priority 7 - Min priority 1 */
    #define INPUT_CAPTURE_LEVEL 7
    #define PWM_TIMER_LEVEL 7
    #define ADC_DMA_LEVEL 6
    #define UART_RX_LEVEL 6
    #define SYS_TIMER_LEVEL 5
    #define CURR_PID_LEVEL 4
    #define VEL_PID_LEVEL 3
    #define DEAD_RECK_LEVEL 2
    #define UART_TX_LEVEL 1
    #define RX_PARSER_LEVEL 1
    #define TIMER3_PRIORITY_LEVEL 1
    
    /* Microcontroller MIPs (FCY) */
    #define SYS_FREQ        80000000
    #define FCY             SYS_FREQ/2

    #define FRTMR1 1000             // Timer1 - Value in herz [Hz]
    #define TCTMR1 1/FRTMR1         // Timer1 - Value in seconds [s]
    #define TMR1_VALUE FCY/FRTMR1   // Timer1 - Value in CLK
    #define TMR2_VALUE 0xFFFF       // Timer2 - Value for overflow

    #define TMR3_VALUE 2000       // Timer3 - 50uSec Overflow
    

//    //Blink LED
//    /**
//     * BL = 0.5 = 1/2
//     * BLINKSW = BL/0.001 = (1/2)/10^-3 = 10^3/2 = 1000/2 = 500
//     */
//    #define BLINK_LED 0.5 //Value in seconds [s]
//    #define BLINKSW (int)(BLINK_LED/TCTMR1)

    //UART
    #define BAUDRATE 115200
    //#define BAUDRATE 57600
    #define BRGVAL   ((FCY/BAUDRATE)/16)-1

    // Current ADC buffer dimension
    #define ADC_CHANNELS 2
    #define ADC_BUFF 64
    #define TOT_ADC_BUFF ADC_CHANNELS * ADC_BUFF

/******************************************************************************/
/* System Function Prototypes                                                 */
/******************************************************************************/

/* Custom oscillator configuration funtions, reset source evaluation
functions, and other non-peripheral microcontroller initialization functions
go here. */

    /**
     * Initialization name process and set standard priority for all procesees
     */
    void init_process(void);

    /**
     * From name recived, return a process required.
     * @param number name process
     * @return save in process_buffer name associated for process
     */
    process_buffer_t decodeNameProcess(int number);

    /**
     * Update priority for process, restart function Init Interrupt for restart
     * process with correct value
     * @return ACK value for correct update priority
     */
    unsigned char update_priority(void);

    /**
     * Upgdate frequency for working processes. If value is equal to zero,
     * the process are disabled.
     * @return ACK value for correct update priority
     */
    unsigned char update_frequency(void);

    /**
     * Managment services messages. Return a service message for correct parsing
     * @param service to parsing
     * @return a new service message
     */
    services_t services(services_t service);

    /** Handles clock switching/osc initialization
     */
    void ConfigureOscillator(void);

    /** Initialization PWM for drive engine left & right
     */
    void InitPWM(void);

    /** Initialization QEI - Motor left
     */
    void InitQEI1(void);

    /** Initialization QEI - Motor right
     */
    void InitQEI2(void);

    /** Initialization Input Capture - Motor left
     */
    void InitIC1(void);

    /** Initialization Input Capture - Motor right
     */
    void InitIC2(void);

    /** Initialization Timer 1 - Timer system
     */
    void InitTimer1(void);

    /** Initalization Timer 2 for IC (Input Capture)
     */
    void InitTimer2(void);
    
    /** Initalization Timer 3
     */
    void InitTimer3(void);

    /** Initalization others interrupts
     */
    void InitInterrupts(void);

    /** Initialization UART1 for comunication
     */
    void InitUART1(void);

    /** Initialization DMA0 for ADC current
     */
    void InitDMA0(void);

    /** Initialization DMA1 for UART Tx transmition
     */
    void InitDMA1(void);

    /** Initialization ADC for measure current motors
     */
    void InitADC(void);

#ifdef	__cplusplus
}
#endif

#endif	/* SYSTEM_H */