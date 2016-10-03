/*
 * Copyright (C) 2014-2015 Officine Robotiche
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
    
#include <system/events.h>

/******************************************************************************/
/* System Level #define Macros                                                */
/******************************************************************************/
    
    /* Microcontroller MIPs (FCY) */
#define FOSC        80000000
#define FCY         (FOSC/2)

#define FRTMR1 10000            // Timer1 - Value in herz [Hz]
#define TCTMR1 1/FRTMR1         // Timer1 - Value in seconds [s]
#define TMR1_VALUE FCY/FRTMR1   // Timer1 - Value in CLK
#define FRTMR2 FOSC
#define TMR2_VALUE 0xFFFF       // Timer2 - Value for overflow
    
    /* Interrupt priority */
    /* Max priority 7 - Min priority 1 */
#define UART_RX_LEVEL 7
#define PWM_TIMER_LEVEL 7
#define INPUT_CAPTURE_LEVEL 6
#define ADC_DMA_LEVEL 6
#define SYS_TIMER_LEVEL 5
    
#define EVENT_PRIORITY_HIGH_LEVEL 4
#define EVENT_PRIORITY_MEDIUM_LEVEL 3
#define EVENT_PRIORITY_LOW_LEVEL 2
#define EVENT_PRIORITY_VERY_LOW_LEVEL 1

#define UART_TX_LEVEL 1
    
    // Definition events
    
    #define NUM_SYSTEM_EVENTS 5

    typedef enum {
        SYSTEM_EVENT_PARSER = 0,
        SYSTEM_EVENT_I2C    = 1,
        SYSTEM_EVENT_LED    = 2
    } system_event_type_t;

    /******************************************************************************/
    /* System Function Prototypes                                                 */
    /******************************************************************************/

    /* Custom oscillator configuration functions, reset source evaluation
    functions, and other non-peripheral microcontroller initialization functions
    go here. */

    /** 
     * Handles clock switching/osc initialization
     */
    void ConfigureOscillator(void);
    
    /**
     * Initialization all system events
     */
    void InitEvents(void);
    
    /** Initialization Timer 1 - Timer system
     */
    void InitTimer1(void);
    
    /**
     * Update time for ADC conversion
     * @param t2
     * @param t1
     */
    void update_adc_time(uint16_t t2, uint16_t t1);
    /**
     * Register system events
     * @param event_type type of event
     * @param event 
     */
    void register_time(system_event_type_t event_type, hEvent_t event );
    
    /**
     * Return the time of execution of idle, parsing, adc conversion
     * @param message
     */
    void get_system_time(message_abstract_u *message);
    /**
     * Board software reset.
     * Disable all interrupt, wait 200us and reset the board
     */
    void reset();

    /**
     * Management services messages. 
     * @param The name of the service
     * @param The buffer to return the information
     */
    void services(unsigned char command, message_abstract_u *message);

#ifdef	__cplusplus
}
#endif

#endif	/* SYSTEM_H */