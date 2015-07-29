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
#include "system/system_comm.h"
#include "system/peripherals.h"

#include "communication/I2c.h"
#include <peripherals/i2c/i2c.h>

#include "communication/serial.h"

#include "motors/motor_init.h"
#include "motors/motor_control.h"
#include "motors/motor_comm.h"

#include "high_control/manager.h"
#include "high_control/high_comm.h"

// high level include
#include "high_control/cartesian.h"
#include "high_control/linefollower.h"

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
 * dynamically, optimizing the resources at most. A slow procedure can be
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

bool ccc = false;
bool bbb = false;
bool aaa = false;
uint8_t rdBuffer[2] = {100, 205};
uint16_t address = 1000;
uint16_t rdSize = 2;

void pCallback(bool b) {
    bbb = b;
}

void wCallback(bool b) {
    aaa = b;
}

int16_t main(void) {
    /** INITIALIZATION Operative System **/
    ConfigureOscillator();  ///< Configure the oscillator for the device
    Peripherals_Init();     ///< Initialize IO ports and peripherals
    InitEvents();   ///< Initialize processes controller
    InitTimer1();   ///< Open Timer1 for clock system
    
    InitLEDs();             ///< Initialization LEDs
    
    /* Peripherals initialization */
    InitTimer2(); ///< Open Timer2 for InputCapture 1 & 2
    InitADC();    ///< Open ADC for measure current motors
    InitDMA0();   ///< Open DMA0 for buffering measures ADC
    
    /* I2C CONFIGURATION */
    Init_I2C();     ///< Open I2C module
    EEPROM_init();  ///< Launch the EEPROM controller
    
    /** SERIAL CONFIGURATION **/
    SerialComm_Init();  ///< Open UART1 for serial communication and Open DMA1 for TX UART1
    set_frame_reader(HASHMAP_SYSTEM, &send_frame_system, &save_frame_system); ///< Initialize parsing reader
    
    /*** MOTOR INITIALIZATION ***/
    InitPWM();      ///< Open PWM
    int i;
    for (i = 0; i < NUM_MOTORS; ++i) {
        Motor_Init(i);                                          ///< Initialization Motor peripherals
        update_motor_parameters(i, init_motor_parameters());    ///< Initialize parameters for motors
        update_motor_pid(i, init_motor_pid());                  ///< Initialize PID controllers
        update_motor_emergency(i, init_motor_emergency());      ///< Initialize emergency procedure to stop
        update_motor_constraints(i, init_motor_constraints());  ///< Initialize constraints motor
        set_motor_state(i, STATE_CONTROL_DISABLE);              ///< Initialize state controller
    }
    set_frame_reader(HASHMAP_MOTOR, &send_frame_motor, &save_frame_motor);  ///< Initialize communication
    
    /** HIGH LEVEL INITIALIZATION **/
    /// Initialize variables for unicycle 
    update_motion_parameter_unicycle(init_motion_parameter_unicycle());
    /// Initialize dead reckoning
    update_motion_coordinate(init_motion_coordinate());
    /// Initialize motion parameters and controller
    HighControl_Init();
    /// Initialize communication
    set_frame_reader(HASHMAP_MOTION, &send_frame_motion, &save_frame_motion);
    
    /* LOAD high level task */
    //add_task(false, &init_cartesian, &loop_cartesian);
    InitTimer3(); // 50uSe Linefollower sensor timebase
    /* LOAD high level task */
    add_task(false, &init_linefollower, &loop_linefollower);

    if(EEPROM_read(0, &rdBuffer[0], address, rdSize, pCallback)) {
        int a;
        a= 1;
    } else {
        int b;
        b= 1;
    }
    uint8_t wrBuffer[2] = {2, 88};

    while (true) {
        PCF8574_LED_write(PCF8574_LED1 + PCF8574_LED5);
        
        /*if(bbb) {
            EEPROM_write(0, wrBuffer, address, 2, wCallback);
            bbb = false;
        }
        if(aaa) {
            EEPROM_read(0, &rdBuffer[0], address, rdSize, pCallback);
            aaa = false;
        }
        if(ccc) {
            EEPROM_service_trigger();
        }*/
    }

    return true;
}
