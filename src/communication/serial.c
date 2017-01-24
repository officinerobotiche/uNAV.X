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

#include <xc.h>                 /* Device header file */

#include <or_system/events.h>

#include "communication/serial.h"

#include "system/system.h"

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

//UART
#define BAUDRATE 115200
#define BRGVAL   ((FCY/BAUDRATE)/16)-1

UART_t *_UART1_CNT;
OR_BUS_FRAME_t *_OR_BUS_FRAME_CNT;
/*! Array for DMA UART buffer */
unsigned char BufferTx[OR_BUS_FRAME_LNG_FRAME] __attribute__((space(dma)));

/******************************************************************************/
/* Communication Functions                                                    */
/******************************************************************************/

void UART1_conf() {
    U1MODEbits.STSEL = 0; // 1-stop bit
    U1MODEbits.PDSEL = 0; // No Parity, 8-data bits
    U1MODEbits.ABAUD = 0; // Auto-Baud Disabled
    U1MODEbits.BRGH = 0; // Low Speed mode

    U1BRG = ((FCY/BAUDRATE)/16)-1; // BAUD Rate Setting on System.h

    U1STAbits.UTXISEL0 = 0; // Interrupt after one Tx character is transmitted
    U1STAbits.UTXISEL1 = 0;

    IEC0bits.U1TXIE = 0; // Disable UART Tx interrupt
    U1STAbits.URXISEL = 0; // Interrupt after one RX character is received

    U1MODEbits.UARTEN = 1; // Enable UART
    U1STAbits.UTXEN = 1; // Enable UART Tx

    IEC4bits.U1EIE = 0;
    IPC2bits.U1RXIP = UART_RX_LEVEL; // Set UART Rx Interrupt Priority Level
    IFS0bits.U1RXIF = 0; // Reset RX interrupt flag
    IEC0bits.U1RXIE = 1; // Enable RX interrupt
}

void UART1_DMA1_conf(void) {
    //DMA1CON = 0x2001;			// One-Shot, Post-Increment, RAM-to-Peripheral

    DMA1CONbits.CHEN = 0;
    DMA1CONbits.SIZE = 1;
    DMA1CONbits.DIR = 1;
    DMA1CONbits.HALF = 0;
    DMA1CONbits.NULLW = 0;
    DMA1CONbits.AMODE = 0;
    DMA1CONbits.MODE = 1;

    //DMA1CNT = MAX_BUFF_TX - 1; // 32 DMA requests
    DMA1REQ = 0x000c; // Select UART1 Transmitter

    DMA1STA = __builtin_dmaoffset(BufferTx);
    DMA1PAD = (volatile unsigned int) &U1TXREG;

    IPC3bits.DMA1IP = UART_TX_LEVEL; // Set DMA Interrupt Priority Level
    IFS0bits.DMA1IF = 0; // Clear DMA Interrupt Flag
    IEC0bits.DMA1IE = 1; // Enable DMA interrupt
}

void UART1_Init(UART_t *uart, OR_BUS_FRAME_t *frame) {
    // Save the pointer
    _UART1_CNT = uart;
    _OR_BUS_FRAME_CNT = frame;
    // Initialization UART 1
    UART1_conf();
    UART1_DMA1_conf();
}

void UART1_DMA_write(unsigned char* buff, size_t size) {
    //Wait to complete send packet from UART1 and DMA1.
    while ((U1STAbits.TRMT == 0) && (DMA1CONbits.CHEN == 0));
    //Copy the message on Buffer DMA TX
    memcpy(&BufferTx, buff, size);
    // Initialize the DMA controller
    DMA1CNT = size - 1; // # of DMA requests
    DMA1CONbits.CHEN = 1; // Enable DMA1 Channel
    DMA1REQbits.FORCE = 1; // Manual mode: Kick-start the 1st transfer
}

void UART1_read_callback(UART_status_type_t type, unsigned char rxdata) {
    // decode message from  UART and parse
    switch(type) {
        case UART_DONE:     // If data DONE launch decoder function
        {
            OR_BUS_State_t state = OR_BUS_FRAME_decoder(_OR_BUS_FRAME_CNT, rxdata);
            switch (state) {
                case OR_BUS_PENDING:
                    // initialize and run timer
                    break;
                case OR_BUS_DONE:
                    // Event decoded message
                    // Build the message and send
                    if(OR_BUS_FRAME_build(_OR_BUS_FRAME_CNT)) {
                        // Send the message
                        UART_write(_UART1_CNT, _OR_BUS_FRAME_CNT->or_bus.tx.buff, 
                                _OR_BUS_FRAME_CNT->or_bus.tx.length);
                    }
                    break;
                default:
                    break;
            }
            break;
        }
        default:            // All error reset the parser
            OR_BUS_FRAME_reset(_OR_BUS_FRAME_CNT);
            break;
    }
    // Reset timer
}
/**
 * @brief UART1 Interrupt
 */
void __attribute__((interrupt, auto_psv)) _U1RXInterrupt(void) {
    UART_read(_UART1_CNT);
}
/**
 * When receive the DMA interrupt unlock the UART writer
 */
void __attribute__((interrupt, no_auto_psv)) _DMA1Interrupt(void) {
    IFS0bits.DMA1IF = 0; // Clear the DMA1 Interrupt Flag
    // Flush buffer
    UART_write_flush_buffer(_UART1_CNT);
}