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

#include <system/events.h>
#include <system/task_manager.h>

#include "system/peripherals.h"
#include "system/system.h"   /* variables/params used by system.c             */

///////  TO REMOVE!!
#include "packet/packet.h"
#include "packet/frame_motion.h"
#include "communication/serial.h"
#include "motors/motor_control.h"
#include "high_control/manager.h"

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

unsigned int reset_count = 0;
system_parameter_t parameter_system;

//process_t default_process[NUM_PROCESS_DEFAULT];
//process_t motor_process[PROCESS_MOTOR_LENGTH];
//process_t motion_process[PROCESS_MOTION_LENGTH];

/******************************************************************************/
/* NEW Global Variable Declaration                                            */
/******************************************************************************/

unsigned char _VERSION_DATE[] = __DATE__;
unsigned char _VERSION_TIME[] = __TIME__;
unsigned char _VERSION_CODE[] = "v0.5";
unsigned char _AUTHOR_CODE[] = "Officine Robotiche";
unsigned char _BOARD_TYPE[] = "Motor Control";
#ifdef UNAV_V1
unsigned char _BOARD_NAME[] = "uNAV";
#elif ROBOCONTROLLER_V3
unsigned char _BOARD_NAME[] = "RoboController";
#elif MOTION_CONTROL
unsigned char _BOARD_NAME[] = "Motion Control";
#endif


uint16_t FRQ_CPU = FRTMR1;

#define EVENT_PRIORITY_LOW_ENABLE IEC3bits.RTCIE
#define EVENT_PRIORITY_LOW_FLAG IFS3bits.RTCIF
#define EVENT_PRIORITY_LOW_P IPC15bits.RTCIP
hardware_bit_t RTCIF = {&IFS3, 14};

#define EVENT_PRIORITY_MEDIUM_ENABLE IEC0bits.OC1IE
#define EVENT_PRIORITY_MEDIUM_FLAG IFS0bits.OC1IF
#define EVENT_PRIORITY_MEDIUM_P IPC0bits.OC1IP
hardware_bit_t OC1IF = {&IFS0, 2};

#define EVENT_PRIORITY_HIGH_ENABLE IEC0bits.OC2IE
#define EVENT_PRIORITY_HIGH_FLAG IFS0bits.OC2IF
#define EVENT_PRIORITY_HIGH_P IPC1bits.OC2IP
hardware_bit_t OC2IF = {&IFS0, 6};

#define EVENT_PRIORITY_VERY_LOW_ENABLE IEC1bits.OC3IE
#define EVENT_PRIORITY_VERY_LOW_FLAG IFS1bits.OC3IF
#define EVENT_PRIORITY_VERY_LOW_P IPC6bits.OC3IP
hardware_bit_t OC3IF = {&IFS1, 9};

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

inline uint16_t get_Frequency(void) {
    return FRQ_CPU;
}

void InitEvents(void) {
    init_events(&TMR1, &PR1);
    
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
    /// Blink controller for all LEDs
    ControllerBlink();
    IFS0bits.T1IF = 0; // Clear Timer 1 Interrupt Flag
}

void set_process(uint8_t command, system_task_t process_state) {
//    if (process_state.hashmap == HASHMAP_SYSTEM) {
//        switch (command) {
//            case SYSTEM_TASK_TIME:
//                default_process[process_state.number].time = process_state.data;
//                break;
//            case SYSTEM_TASK_PRIORITY:
//                default_process[process_state.number].priority = process_state.data;
//                break;
//            case SYSTEM_TASK_FRQ:
//                default_process[process_state.number].frequency = process_state.data;
//                break;
//        }
//    } else if (process_state.hashmap == HASHMAP_MOTOR) {
//        switch (command) {
//            case SYSTEM_TASK_TIME:
//                motor_process[process_state.number].time = process_state.data;
//                break;
//            case SYSTEM_TASK_PRIORITY:
//                motor_process[process_state.number].priority = process_state.data;
//                break;
//            case SYSTEM_TASK_FRQ:
//                motor_process[process_state.number].frequency = process_state.data;
//                break;
//        }
//    } else if (process_state.hashmap == HASHMAP_MOTION) {
//        switch (command) {
//            case SYSTEM_TASK_TIME:
//                motion_process[process_state.number].time = process_state.data;
//                break;
//            case SYSTEM_TASK_PRIORITY:
//                motion_process[process_state.number].priority = process_state.data;
//                break;
//            case SYSTEM_TASK_FRQ:
//                motion_process[process_state.number].frequency = process_state.data;
//                break;
//        }
//    }
}

system_task_t get_process(uint8_t command, system_task_t process_state) {
//    if (process_state.hashmap == HASHMAP_SYSTEM) {
//        switch (command) {
//            case SYSTEM_TASK_TIME:
//                process_state.data = default_process[process_state.number].time;
//                break;
//            case SYSTEM_TASK_PRIORITY:
//                process_state.data = default_process[process_state.number].priority;
//                break;
//            case SYSTEM_TASK_FRQ:
//                process_state.data = default_process[process_state.number].frequency;
//                break;
//            case SYSTEM_TASK_NUM:
//                process_state.data = NUM_PROCESS_DEFAULT;
//                break;
//        }
//    } else if (process_state.hashmap == HASHMAP_MOTOR) {
//        switch (command) {
//            case SYSTEM_TASK_TIME:
//                process_state.data = motor_process[process_state.number].time;
//                break;
//            case SYSTEM_TASK_PRIORITY:
//                process_state.data = motor_process[process_state.number].priority;
//                break;
//            case SYSTEM_TASK_FRQ:
//                process_state.data = motor_process[process_state.number].frequency;
//                break;
//            case SYSTEM_TASK_NUM:
//                process_state.data = PROCESS_MOTOR_LENGTH;
//                break;
//        }
//    } else if (process_state.hashmap == HASHMAP_MOTION) {
//        switch (command) {
//            case SYSTEM_TASK_TIME:
//                process_state.data = motion_process[process_state.number].time;
//                break;
//            case SYSTEM_TASK_PRIORITY:
//                process_state.data = motion_process[process_state.number].priority;
//                break;
//            case SYSTEM_TASK_FRQ:
//                process_state.data = motion_process[process_state.number].frequency;
//                break;
//            case SYSTEM_TASK_NUM:
//                process_state.data = PROCESS_MOTION_LENGTH;
//                break;
//        }
//    }
    return process_state;
}

system_task_name_t get_process_name(system_task_name_t process_name) {
//    if (process_name.hashmap == HASHMAP_SYSTEM) {
//        strcpy(process_name.data, default_process[process_name.number].name);
//    } else if (process_name.hashmap == HASHMAP_MOTOR) {
//        strcpy(process_name.data, motor_process[process_name.number].name);
//    } else if (process_name.hashmap == HASHMAP_MOTION) {
//        strcpy(process_name.data, motion_process[process_name.number].name);
//    }
    return process_name;
}

//unsigned char update_priority(void) {
//    default_process[PROCESS_IDLE].time = 0;
//    InitInterrupts();
//    return PACKET_ACK;
//}

unsigned char update_frequency(void) {
//    if (motor_process[LEFT_PROCESS_PID].frequency == 0 || motor_process[RIGHT_PROCESS_PID].frequency == 0) {
//        EVENT_PRIORITY_MEDIUM_ENABLE = 0; // Disable Output Compare Channel 1 interrupt
//    } else
//        EVENT_PRIORITY_MEDIUM_ENABLE = 1; // Enable Output Compare Channel 1 interrupt
//    if (motion_process[PROCESS_ODOMETRY].frequency == 0 || motion_process[PROCESS_VELOCITY].frequency == 0) {
//        EVENT_PRIORITY_LOW_ENABLE = 0; // Disable RTC interrupt
//    } else
//        EVENT_PRIORITY_LOW_ENABLE = 1; // Enable RTC interrupt
    return PACKET_ACK;
}

system_service_t services(system_service_t service) {
    system_service_t service_send;
    service_send.command = service.command;
    switch (service.command) {
        case SERVICE_CODE_DATE:
            memcpy(service_send.buffer, _VERSION_DATE, sizeof (_VERSION_DATE));
            service_send.buffer[sizeof (_VERSION_DATE) - 1] = ' ';
            memcpy(service_send.buffer + sizeof (_VERSION_DATE), _VERSION_TIME, sizeof (_VERSION_TIME));
            break;
        case SERVICE_CODE_BOARD_NAME:
            memcpy(service_send.buffer, _BOARD_NAME, sizeof (_BOARD_NAME));
            break;
        case SERVICE_CODE_BOARD_TYPE:
            memcpy(service_send.buffer, _BOARD_TYPE, sizeof (_BOARD_TYPE));
            break;
        case SERVICE_CODE_VERSION:
            memcpy(service_send.buffer, _VERSION_CODE, sizeof (_VERSION_CODE));
            break;
        case SERVICE_CODE_AUTHOR:
            memcpy(service_send.buffer, _AUTHOR_CODE, sizeof (_AUTHOR_CODE));
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
