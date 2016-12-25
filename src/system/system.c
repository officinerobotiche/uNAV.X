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

#include <xc.h>              /* Device header file */

#include <stdint.h>          /* For uint16_t definition                       */
#include <stdbool.h>         /* For true/false definition                     */
#include <pwm12.h>
#include <string.h>

#include <or_system/events.h>
#include <or_system/task_manager.h>

#include "system/system.h"   /* variables/params used by system.c             */
#include "communication/serial.h"

#include <libpic30.h>    /* Inclusion for delay REQUIRED after definition FCY */

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

//system_parameter_t parameter_system = {(frequency_t) SYS_FREQ, (frequency_t) FRTMR1};

const unsigned char _VERSION_DATE[] = __DATE__;
const unsigned char _VERSION_TIME[] = __TIME__;
const unsigned char _VERSION_CODE[] = "v0.6";
const unsigned char _AUTHOR_CODE[] = "Officine Robotiche";
const unsigned char _BOARD_TYPE[] = "Motor Control";
#ifdef UNAV_V1
const unsigned char _BOARD_NAME[] = "uNAV";
#elif ROBOCONTROLLER_V3
const unsigned char _BOARD_NAME[] = "RoboController";
#elif MOTION_CONTROL
const unsigned char _BOARD_NAME[] = "Motion Control";
#endif

#define EVENT_PRIORITY_LOW_ENABLE IEC3bits.RTCIE
#define EVENT_PRIORITY_LOW_FLAG IFS3bits.RTCIF
#define EVENT_PRIORITY_LOW_P IPC15bits.RTCIP
hardware_bit_t RTCIF = REGISTER_INIT(IFS3, 14);

#define EVENT_PRIORITY_MEDIUM_ENABLE IEC0bits.OC1IE
#define EVENT_PRIORITY_MEDIUM_FLAG IFS0bits.OC1IF
#define EVENT_PRIORITY_MEDIUM_P IPC0bits.OC1IP
hardware_bit_t OC1IF = REGISTER_INIT(IFS0, 2);

#define EVENT_PRIORITY_HIGH_ENABLE IEC0bits.OC2IE
#define EVENT_PRIORITY_HIGH_FLAG IFS0bits.OC2IF
#define EVENT_PRIORITY_HIGH_P IPC1bits.OC2IP
hardware_bit_t OC2IF = REGISTER_INIT(IFS0, 6);

#define EVENT_PRIORITY_VERY_LOW_ENABLE IEC1bits.OC3IE
#define EVENT_PRIORITY_VERY_LOW_FLAG IFS1bits.OC3IF
#define EVENT_PRIORITY_VERY_LOW_P IPC6bits.OC3IP
hardware_bit_t OC3IF = REGISTER_INIT(IFS1, 9);

uint32_t adc_time;
hEvent_t system_events[NUM_SYSTEM_EVENTS];

/******************************************************************************/
/* System Level Functions                                                     */
/******************************************************************************/

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

void InitEvents(void) {
    /// Register event controller
    init_events(&TMR1, &PR1, FOSC, 6);
    
    EVENT_PRIORITY_VERY_LOW_ENABLE = 0;
    EVENT_PRIORITY_VERY_LOW_P = EVENT_PRIORITY_VERY_LOW_LEVEL;
    register_interrupt(EVENT_PRIORITY_VERY_LOW, &RTCIF);
    EVENT_PRIORITY_VERY_LOW_ENABLE = 1;
    
    EVENT_PRIORITY_LOW_ENABLE = 0;
    EVENT_PRIORITY_LOW_P = EVENT_PRIORITY_LOW_LEVEL;
    register_interrupt(EVENT_PRIORITY_LOW, &RTCIF);
    EVENT_PRIORITY_LOW_ENABLE = 1;
    
    EVENT_PRIORITY_MEDIUM_ENABLE = 0;
    EVENT_PRIORITY_MEDIUM_P = EVENT_PRIORITY_MEDIUM_LEVEL;
    register_interrupt(EVENT_PRIORITY_MEDIUM, &OC1IF);
    EVENT_PRIORITY_MEDIUM_ENABLE = 1;
    
    EVENT_PRIORITY_HIGH_ENABLE = 0;
    EVENT_PRIORITY_HIGH_P = EVENT_PRIORITY_HIGH_LEVEL;
    register_interrupt(EVENT_PRIORITY_HIGH, &OC2IF);
    EVENT_PRIORITY_HIGH_ENABLE = 1;
    
    adc_time = 0;
    /// Initialization task controller
    task_init(FRTMR1);
}

void __attribute__((interrupt, auto_psv)) _RTCCInterrupt(void) {
    event_manager(EVENT_PRIORITY_LOW);
    EVENT_PRIORITY_LOW_FLAG = 0; //interrupt flag reset
}

void __attribute__((interrupt, auto_psv)) _OC1Interrupt(void) {
    event_manager(EVENT_PRIORITY_MEDIUM);
    EVENT_PRIORITY_MEDIUM_FLAG = 0; // interrupt flag reset
}

void __attribute__((interrupt, auto_psv)) _OC2Interrupt(void) {
    event_manager(EVENT_PRIORITY_HIGH);
    EVENT_PRIORITY_HIGH_FLAG = 0; //interrupt flag reset
}

void __attribute__((interrupt, auto_psv)) _OC3Interrupt(void) {
    event_manager(EVENT_PRIORITY_VERY_LOW);
    EVENT_PRIORITY_VERY_LOW_FLAG = 0;
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

void __attribute__((interrupt, auto_psv)) _T1Interrupt(void) {
    /// Execution task manager
    task_manager();
    IFS0bits.T1IF = 0; // Clear Timer 1 Interrupt Flag
}

void update_adc_time(uint16_t t2, uint16_t t1) {
    if(t2 >= t1) {
        adc_time = t2 - t1;
    } else {
        adc_time = t2 + (0xFFFF - t1);
    }
}

void register_time(system_event_type_t event_type, hEvent_t event ) {
    system_events[event_type] = event;
}

void get_system_time(message_abstract_u *message) {
    message->system.time.idle = 0;
    message->system.time.parser = get_time(system_events[SYSTEM_EVENT_PARSER]);
    message->system.time.i2c = get_time(system_events[SYSTEM_EVENT_I2C]);
    message->system.time.led = get_time(system_events[SYSTEM_EVENT_LED]);
    message->system.time.adc = adc_time;
}

//inline system_parameter_t get_system_parameters(void) {
//    return parameter_system;
//}

void reset() {
    // disable all user interrupts
    SET_CPU_IPL(7);
    /* __delay_ms() and __delay_us() are defined as macros. They depend
     * on a user-supplied definition of FCY. If FCY is defined, the argument
     * is converted and passed to __delay32(). Otherwise, the functions
     * are declared external.
     */
    __delay_us(200);
    // System reset
    asm("RESET");
}

void services(unsigned char command, message_abstract_u *message) {
    switch(command) {
        case SYSTEM_CODE_DATE:
            memcpy(message->system.service, _VERSION_DATE, sizeof (_VERSION_DATE));
            message->system.service[sizeof (_VERSION_DATE) - 1] = ' ';
            memcpy(message->system.service + sizeof (_VERSION_DATE), _VERSION_TIME, sizeof (_VERSION_TIME));
            break;
        case SYSTEM_CODE_VERSION:
            memcpy(message->system.service, _VERSION_CODE, sizeof (_VERSION_CODE));
            break;
        case SYSTEM_CODE_AUTHOR:
            memcpy(message->system.service, _AUTHOR_CODE, sizeof (_AUTHOR_CODE));
            break;
        case SYSTEM_CODE_BOARD_TYPE:
            memcpy(message->system.service, _BOARD_TYPE, sizeof (_BOARD_TYPE));
            break;
        case SYSTEM_CODE_BOARD_NAME:
            memcpy(message->system.service, _BOARD_NAME, sizeof (_BOARD_NAME));
            break;
    }
}
