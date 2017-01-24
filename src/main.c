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
/*	CONFIGURATION BITS													      */
/******************************************************************************/
// FBS
#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)
#pragma config RBS = NO_RAM             // Boot Segment RAM Protection (No Boot RAM)
// FSS
#pragma config SWRP = WRPROTECT_OFF     // Secure Segment Program Write Protect (Secure segment may be written)
#pragma config SSS = NO_FLASH           // Secure Segment Program Flash Code Protection (No Secure Segment)
#pragma config RSS = NO_RAM             // Secure Segment Data RAM Protection (No Secure RAM)
// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)
/** 
 * Oscillator selection configuration
 * * FNOSC_PRI -> Primary (XT, HS, EC) Oscillator
 * * IESO_ON -> Start-up device with FRC, then automatically switch to
 * user-selected oscillator source when ready
 */
// FOSCSEL
#pragma config FNOSC = PRI              // Oscillator Mode (Primary Oscillator (XT, HS, EC))
#pragma config IESO = ON                // Internal External Switch Over Mode (Start-up device with FRC, then automatically switch to user-selected oscillator source when ready)
/** Oscillator configuration
 * * FCKSM_CSECME -> Both Clock Switching and Fail-Safe Clock Monitor are enabled
 * * OSCIOFNC_OFF -> OSC2 pin has clock out function
 * * POSCMD_HS -> Primary Oscillator Mode, HS Crystal
*/
// FOSC
#pragma config POSCMD = HS              // Primary Oscillator Source (HS Oscillator Mode)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function (OSC2 pin has clock out function)
#pragma config IOL1WAY = ON             // Peripheral Pin Select Configuration (Allow Only One Re-configuration)
#pragma config FCKSM = CSECME           // Clock Switching and Monitor (Both Clock Switching and Fail-Safe Clock Monitor are enabled)
/** 
 * Watchdog Timer Enabled/disabled by user software
 * (LPRC can be disabled by clearing SWDTEN bit in RCON register
 */
// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler (1:32,768)
#pragma config WDTPRE = PR128           // WDT Prescaler (1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog timer enabled/disabled by user software)
// FPOR
#pragma config FPWRT = PWR128           // POR Timer Value (128ms)
#pragma config ALTI2C = OFF             // Alternate I2C  pins (I2C mapped to SDA1/SCL1 pins)
#pragma config LPOL = ON                // Motor Control PWM Low Side Polarity bit (PWM module low side output pins have active-high output polarity)
#pragma config HPOL = ON                // Motor Control PWM High Side Polarity bit (PWM module high side output pins have active-high output polarity)
#pragma config PWMPIN = ON              // Motor Control PWM Module Pin Mode bit (PWM module pins controlled by PORT register at device Reset)
// FICD
#pragma config ICS = PGD1               // Comm Channel Select (Communicate on PGC1/EMUC1 and PGD1/EMUD1)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG is Disabled)

/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

#include <xc.h>             /* Device header file */

#include <stdint.h>        /* Includes uint16_t definition                    */
#include <stdbool.h>       /* Includes true/false definition                  */

#include "system/system.h" /* System funct/params, like osc/peripheral config */
#include "system/peripherals.h" /* Peripheral configuration */
#include "communication/serial.h" /* Serial port configuration */

#include "motors/motor_init.h"

//#include "system/system_comm.h"
//

//#include "system/peripherals_comm.h"
//
//#include "communication/I2c.h"
//#include <or_peripherals/I2C/MCP24LC256.h>
//
//#include "communication/serial.h"
//#include "motors/motor_control.h"
//#include "motors/motor_comm.h"
//
//#include "high_control/manager.h"
//#include "high_control/high_comm.h"
//
//// high level include
//#include "high_control/cartesian.h"

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

#ifdef UNAV_V1
// Initialization LED on uNAV
LED_t leds[] = {
    GPIO_LED(C, 6), // Led 1 Green
    GPIO_LED(C, 7), // Led 2 Red
    GPIO_LED(C, 8), // Led 3 Yellow
    GPIO_LED(C, 9), // Led 4 Blue
};
#elif ROBOCONTROLLER_V3
// Initialization LED on RoboController
LED_t leds[] = {
    GPIO_LED(A, 8), // Led 1 green
    GPIO_LED(A, 9), // Led 2 green
};
#elif MOTION_CONTROL
// Initialization LED on motion control
LED_t leds[] = {
    GPIO_LED(A, 4), // Led Blue
};
#endif
/// Number of available LEDs
#define LED_NUM (sizeof(leds) / ( sizeof(leds[0])))
// Initialization LED controller
LED_controller_t LED_CONTROLLER = LED_CONTROLLER(leds, LED_NUM, 1000);
/** Define and initialization UART1 controller */
UART_WRITE_t UART1_WRITE_CNT;
UART_READ_t UART1_READ_CNT;
UART_t UART1_CNT = UART_INIT(U1STA, U1MODE, U1BRG, FCY, &UART1_WRITE_CNT, &UART1_READ_CNT);
/** Controller OR BUS messages */
unsigned char OR_BUS_RX_BUFFER[OR_BUS_FRAME_LNG_FRAME];
unsigned char OR_BUS_TX_BUFFER[OR_BUS_FRAME_LNG_FRAME];
OR_BUS_FRAME_t OR_BUS_FRAME;

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
 * @return type of error
 */
int16_t main(void) {
    /** INITIALIZATION Operative System **/
    ConfigureOscillator();  ///< Configure the oscillator for the device
    InitEvents();           ///< Initialize processes controller
    InitTimer1();           ///< Open Timer1 for clock system
    
    Peripherals_Init();     ///< Initialize IO ports and peripherals
    // Initialization LED
    LED_Init(&LED_CONTROLLER);
    
    /* I2C CONFIGURATION */
//    Init_I2C();     ///< Open I2C module
//    EEPROM_init(20);  ///< Launch the EEPROM controller
    
    /** SERIAL CONFIGURATION **/
    // Initialization over bus
    OR_BUS_FRAME_init(&OR_BUS_FRAME, &OR_BUS_TX_BUFFER[0], &OR_BUS_RX_BUFFER[0], OR_BUS_FRAME_LNG_FRAME);
    // Register callback for system messages
    OR_BUS_FRAME_register(&OR_BUS_FRAME, HASHMAP_SYSTEM, &OR_BUS_FRAME_decoder_system, &OR_BUS_FRAME);
    // Register callback for peripheral messages
    //OR_BUS_FRAME_register(&OR_BUS_FRAME, HASHMAP_PERIPHERALS, &OR_BUS_FRAME_decoder_peripheral, NULL);
    // Register callback for motor messages
    OR_BUS_FRAME_register(&OR_BUS_FRAME, HASHMAP_MOTOR, &OR_BUS_FRAME_decoder_motor, &OR_BUS_FRAME);
    // Register callback for differential drive messages
    //OR_BUS_FRAME_register(&OR_BUS_FRAME, HASHMAP_DIFF_DRIVE, &OR_BUS_FRAME_decoder_diff_drive, NULL);
    // Register UART write
    UART_register_write(&UART1_WRITE_CNT, &U1TXREG, &UART1_DMA_write);
    // Register UART read
    UART_register_read(&UART1_READ_CNT, &U1RXREG, &IFS0, 11, &UART1_read_callback);
    // Initialize UART2 
    UART1_Init(&UART1_CNT, &OR_BUS_FRAME);
    
    /*** MOTOR INITIALIZATION ***/
    Motor_Init(&LED_CONTROLLER);
//    
//    /** HIGH LEVEL INITIALIZATION **/
//    /// Initialize variables for unicycle 
//    update_motion_parameter_unicycle(init_motion_parameter_unicycle());
//    /// Initialize dead reckoning
//    update_motion_coordinate(init_motion_coordinate());
//    /// Initialize motion parameters and controller
//    HighControl_Init();

    /* LOAD high level task */
    //add_task(false, &init_cartesian, &loop_cartesian);

    // Events controller    
    EventsController();

    return 0;
}
