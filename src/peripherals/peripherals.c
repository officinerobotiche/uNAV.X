/*
 * Copyright (C) 2014-2017 Officine Robotiche
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

#include <xc.h>

/* Scheduler include files. */
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"

#include <or_bus/frame.h>

#include "peripherals/peripherals.h"
#include "peripherals/serial.h"

#define comSTACK_SIZE				configMINIMAL_STACK_SIZE
#define mainCOM_TEST_PRIORITY				( 2 )

/* Baud rate used by the comtest tasks. */
#define mainCOM_TEST_BAUD_RATE				( 115200 )
/* We should find that each character can be queued for Tx immediately and we
don't have to block to send. */
#define comNO_BLOCK					( ( TickType_t ) 0 )

#define ledSTACK_SIZE				configMINIMAL_STACK_SIZE
#define mainLED_TASK_PRIORITY				( tskIDLE_PRIORITY + 4 )
/* The execution period of the check task. */
#define mainLED_TASK_PERIOD				( ( TickType_t ) 1 / portTICK_PERIOD_MS )

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

/** Controller OR BUS messages */
unsigned char OR_BUS_RX_BUFFER[OR_BUS_FRAME_LNG_FRAME];
unsigned char OR_BUS_TX_BUFFER[OR_BUS_FRAME_LNG_FRAME];
OR_BUS_FRAME_t OR_BUS_FRAME;

// Initialization LED on uNAV
LED_t leds[] = {
    GPIO_LED(C, 6), // Led 1 Green
    GPIO_LED(C, 7), // Led 2 Red
    GPIO_LED(C, 8), // Led 3 Yellow
    GPIO_LED(C, 9), // Led 4 Blue
};
// Initialization LED controller
LED_controller_t LED_CONTROLLER = LED_CONTROLLER(leds, mainLED_TASK_PERIOD);

/******************************************************************************/
/* System Level Functions                                                     */

/******************************************************************************/

static portTASK_FUNCTION(vComRxTask, pvParameters) {
    signed char cByteRxed;
    // Load motor information
    OR_BUS_FRAME_t *frame = (OR_BUS_FRAME_t*) pvParameters;

    for (;;) {
        // If is available a byte decode the message
        if (xSerialGetChar(NULL, &cByteRxed, comNO_BLOCK)) {
            // Decoder characters
            OR_BUS_State_t state = OR_BUS_FRAME_decoder(frame, cByteRxed);
            // Decoder OR_BUS status
            switch (state) {
                case OR_BUS_PENDING:
                    // initialize and run timer
                    break;
                case OR_BUS_DONE:
                    // Event decoded message
                    // Build the message and send
                    if (OR_BUS_FRAME_build(frame)) {
                        // Send the message
//                            UART_write(_UART1_CNT, _OR_BUS_FRAME_CNT->or_bus.tx.buff, 
//                                    _OR_BUS_FRAME_CNT->or_bus.tx.length);
                    }
                    break;
                default:
                    break;
            }
        }
    }
}

static portTASK_FUNCTION(vLEDControllerTask, pvParameters) {
    /* Used to wake the task at the correct frequency. */
    TickType_t xLastExecutionTime;

    /* Initialise xLastExecutionTime so the first call to vTaskDelayUntil()
    works correctly. */
    xLastExecutionTime = xTaskGetTickCount();
    
    for (;;) {
        /* Wait until it is time for the next cycle. */
        vTaskDelayUntil(&xLastExecutionTime, mainLED_TASK_PERIOD);
        // Launch LED BLINK controller
        LED_blinkController(&LED_CONTROLLER);
    }
}

void StartCom1Tasks( UBaseType_t uxPriority, uint32_t ulBaudRate ) {
    // Initialization Serial port 1
    xSerialPortInitMinimal( ulBaudRate, ( UBaseType_t ) 10 );
    // Initialization over bus
    OR_BUS_FRAME_init(&OR_BUS_FRAME, &OR_BUS_TX_BUFFER[0], &OR_BUS_RX_BUFFER[0], OR_BUS_FRAME_LNG_FRAME);
    // Initialize a Task to read runtime all char received
    xTaskCreate( vComRxTask, "COMRx", comSTACK_SIZE, &OR_BUS_FRAME, uxPriority , ( TaskHandle_t * ) NULL );
}

void Peripherals_Init(void) {
    
    // Peripheral PIN re mapping **********************************
    __builtin_write_OSCCONL(OSCCON & 0xbf); // Unlock Registers
    /********************/
    RPINR7bits.IC1R = 10; // IC1 To Pin RP10
    RPINR7bits.IC2R = 6; // IC2 To Pin RP6
    // QEI
    RPINR14bits.QEA1R = 10; // QEA1 To Pin RP10
    RPINR14bits.QEB1R = 11; // QEB1 To Pin RP11
    RPINR16bits.QEA2R = 5; // QEA2 To Pin RP5
    RPINR16bits.QEB2R = 6; // QEB2 To Pin RP6
    // UART
    RPINR18bits.U1RXR = 21; // U1RX To Pin RP21, CTS tied Vss
    RPINR18bits.U1CTSR = 0x1f;
    RPOR10bits.RP20R = 3; // U1Tx To Pin RP20

    RPINR19bits.U2RXR = 3; // U2RX To Pin RP3, CTS tied Vss
    RPINR19bits.U2CTSR = 0x1f;
    RPOR1bits.RP2R = 5; // U2Tx To Pin RP2
    /********************/
    __builtin_write_OSCCONL(OSCCON | 0x40); // Lock Registers
    // *********************************** Peripheral PIN selection
    
    // Initialization LED controller
    LED_Init(&LED_CONTROLLER);
    /* Create the test tasks defined within this file. */
	xTaskCreate( vLEDControllerTask, "LEDCnt", ledSTACK_SIZE, NULL, mainLED_TASK_PRIORITY, NULL );
    // Initialization Serial port COM1
    StartCom1Tasks( mainCOM_TEST_PRIORITY, mainCOM_TEST_BAUD_RATE );
}

inline void LED_update(short num, short blink) {
    // Wrap to LED blinker controller
    LED_updateBlink(&LED_CONTROLLER, num, blink);
}