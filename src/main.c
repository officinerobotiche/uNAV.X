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

#include <stdint.h>        /* Includes uint16_t definition                    */
#include <stdbool.h>       /* Includes true/false definition                  */

#include "system/system.h" /* System funct/params, like osc/peripheral config */
#include "system/user.h"   /* User funct/params, such as InitApp              */

#include "communication/serial.h"

#include "motors/motor_init.h"
#include "motors/motor_control.h"
#include "motors/motor_comm.h"

#include "high_control/manager.h"
#include "high_control/high_comm.h"

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

/** Main Program
 * The uNav board is designed with one dsPIC33FJ64MC804 to both control the
 * motors and perform navigation.
 * The program is fully interrupt driven.
 * After the initializations, the program enters in a "no-code" Main loop.
 * Every action is started via Interrupt Service Routines triggered by
 * interrupts.
 * The dsPIC33F has 44 interrupt vectors used by the peripherals and
 * 8 reserved to the system.
 * Some interrupts are managed directly by the hardware peripherals or through
 * the DMA, other are used as "soft interrupts" triggered by the code.
 * In this way it's possible to define the priority for each function, even
 * dinamically, optimizing the resources at most. A slow procedure can be
 * interrupted by time-critical one performing a true real-time behavior.
 *
 * Let's analyzing in detail the ISRs
 * Peripheral interrupts:
 * - Input Capture 1 and 2 used to obtain the speed;
 * - Timer 1 overflow used as the time scheduler for all the timed procedures;
 * - Timer 2 overflow used, together with IC1 and IC2, to measure the
 * encoder ticks;
 * - UART1 RX for incoming communication;
 * - DMA0 used by the ADC to measure the motor current;
 * - DMA1 used by UART TX.
 *
 * Soft interrupts:
 * - OC1 triggers the speed measurement and PIDs control;
 * - OC2 triggers the incoming communication packets parsing;
 * - RTC triggers the dead-reckoning procedures.
 * @return type of error
 */

int16_t main(void) {
    int i;
    /* Configure the oscillator for the device */
    ConfigureOscillator();
    /* Initialize processes controller */
    init_process();
    /* Initialize IO ports and peripherals */
    InitApp();
    
    /** SERIAL CONFIGURATION **/
    /* Initialize hashmap packet */
    init_hashmap_packet();
    /* Initialize buffer serial error */
    init_buff_serial_error();
    /* Initialize parsing reader */
    set_frame_reader(HASHMAP_SYSTEM, &save_frame_system, &send_frame_system);
    
    /*** MOTOR INITIALIZATION ***/
    /* Open PWM */
    InitPWM();
    for (i = 0; i < NUM_MOTORS; ++i) {
        /* Open QEI */
        InitQEI(i);
        /* Open Input Capture */
        InitIC(i);
        /* Initialize variables for motors */
        init_motor(i);
        /* Initialize parameters for motors */
        update_motor_parameters(i, init_motor_parameters());
        /* Initialize PID controllers */
        update_motor_pid(i, init_motor_pid());
        /* Initialize emergency procedure to stop */
        update_motor_emergency(i, init_motor_emergency());
        /* Initialize constraints motor */
        update_motor_constraints(i, init_motor_constraints());
        /* Initialize state controller */
        set_motor_state(i, STATE_CONTROL_DISABLE);
    }
    /* Initialize communication */
    set_frame_reader(HASHMAP_MOTOR, &save_frame_motor, &send_frame_motor);
    
    /** HIGH LEVEL INITIALIZATION **/
    /* Initialize variables for unicycle */
    init_parameter_unicycle();
    /* Initialize dead reckoning */
    init_coordinate();
    /* Initialize communication */
    set_frame_reader(HASHMAP_MOTION, &save_frame_motion, &send_frame_motion);
    
    while (1) {

    }

    return 1;
}
