/*
 µNAV v 0.0.0 alpha 15/09/2014

 Processor : DSPIC33FJ128MC804 clock 40 MHz

 Authors:
 Guido Ottaviani
 Marco d'Ambrosio
 Raffaello Bonghi
 Roberto D'Amico

 License MIT

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
#include "communication/parsing_packet.h"
#include "control/motors_PID.h"
#include "control/high_level_control.h"

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

/* i.e. uint16_t <variable_name>; */

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

    /* Configure the oscillator for the device */
    ConfigureOscillator();

    /* Initialize hashmap packet */
    init_hashmap();
    /* Initialize variables for robots */
    init_buff_serial_error();
    init_parameter();
    init_process();
    init_pid_control();

    /* Initialize pid controllers */
    update_pid_l();
    update_pid_r();

    /* Initialize dead reckoning */
    init_coordinate();

    /* Initialize IO ports and peripherals */
    InitApp();

    /* Start LED blink show */
    blinkflush();

    while (1) {

    }

    return 1;
}
