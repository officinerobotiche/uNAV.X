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

#include <packet/packet.h>

    /******************************************************************************/
    /* System Level #define Macros                                                */
    /******************************************************************************/

    /* Interrupt priority */
    /* Max priority 7 - Min priority 1 */
#define PWM_TIMER_LEVEL 7
#define INPUT_CAPTURE_LEVEL 6
#define ADC_DMA_LEVEL 6
#define UART_RX_LEVEL 6
#define SYS_TIMER_LEVEL 5
    
#define EVENT_PRIORITY_HIGH_LEVEL 4
#define EVENT_PRIORITY_MEDIUM_LEVEL 3
#define EVENT_PRIORITY_LOW_LEVEL 2
//#define MEASURE_LEVEL 4
//#define CURR_PID_LEVEL 4
//#define VEL_PID_LEVEL 3
//#define DEAD_RECK_LEVEL 2
    
#define UART_TX_LEVEL 1
//#define RX_PARSER_LEVEL 1

    /* Microcontroller MIPs (FCY) */
#define SYS_FREQ        80000000
#define FCY             SYS_FREQ/2

#define FRTMR1 1000             // Timer1 - Value in herz [Hz]
#define TCTMR1 1/FRTMR1         // Timer1 - Value in seconds [s]
#define TMR1_VALUE FCY/FRTMR1   // Timer1 - Value in CLK
#define FRTMR2 SYS_FREQ
#define TMR2_VALUE 0xFFFF       // Timer2 - Value for overflow

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

#define NUM_PROCESS_DEFAULT 2
#define PROCESS_IDLE 0
#define PROCESS_PARSE 1

    typedef struct process {
        char name[MAX_BUFF_TASK_NAME];
        uint8_t time;
        uint8_t priority;
        uint8_t frequency;
    } process_t;

    /******************************************************************************/
    /* System Function Prototypes                                                 */
    /******************************************************************************/

    /* Custom oscillator configuration functions, reset source evaluation
    functions, and other non-peripheral microcontroller initialization functions
    go here. */

    /**
     * Initialization led blink
     */
    void InitLEDs(void);
    /**
     * Update frequency or type of blink
     * @param led to control
     * @param blink number of blinks
     */
    void UpdateBlink(short num, short blink);
    
    /**
     * Initialization all system events
     */
    void InitEvents(void);
    
    /**
     * Initialization name process and set standard priority for all procesees
     */
    //void init_process(void);
    /** Initialization others interrupts
     */
    //void InitInterrupts(void);


    /**
     * Update priority for process, restart function Init Interrupt for restart
     * process with correct value
     * @return ACK value for correct update priority
     */
    unsigned char update_priority(void);

    /**
     * Update frequency for working processes. If value is equal to zero,
     * the process are disabled.
     * @return ACK value for correct update priority
     */
    unsigned char update_frequency(void);

    void set_process(uint8_t command, system_task_t process_state);
    /**
     * From name received, return a process required.
     * @param number name process
     * @return save in process_buffer name associated for process
     */
    system_task_name_t get_process_name(system_task_name_t process_state);
    system_task_t get_process(uint8_t command, system_task_t process_state);

    /**
     * Management services messages. Return a service message for correct parsing
     * @param service to parsing
     * @return a new service message
     */
    system_service_t services(system_service_t service);

    /** Handles clock switching/osc initialization
     */
    void ConfigureOscillator(void);

    /** Initialization Timer 1 - Timer system
     */
    void InitTimer1(void);

    /** Initialization Timer 2 for IC (Input Capture)
     */
    void InitTimer2(void);

    /** Initialization DMA0 for ADC current
     */
    void InitDMA0(void);

    /** Initialization ADC for measure current motors
     */
    void InitADC(void);

#ifdef	__cplusplus
}
#endif

#endif	/* SYSTEM_H */