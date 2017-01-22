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

#include "peripherals/serial.h"

#include <string.h>

/* Hardware setup. */
#define serOUTPUT						0
#define serINPUT						1
#define serLOW_SPEED					0
#define serONE_STOP_BIT					0
#define serEIGHT_DATA_BITS_NO_PARITY	0
#define serNORMAL_IDLE_STATE			0
#define serAUTO_BAUD_OFF				0
#define serLOOPBACK_OFF					0
#define serWAKE_UP_DISABLE				0
#define serNO_HARDWARE_FLOW_CONTROL		0
#define serSTANDARD_IO					0
#define serNO_IRDA						0
#define serCONTINUE_IN_IDLE_MODE		0
#define serUART_ENABLED					1
#define serINTERRUPT_ON_SINGLE_CHAR		0
#define serTX_ENABLE					1
#define serINTERRUPT_ENABLE				1
#define serINTERRUPT_DISABLE			0
#define serCLEAR_FLAG					0
#define serSET_FLAG						1

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

/*! Array for DMA UART buffer */
unsigned char BufferTx[200] __attribute__((space(dma)));

/* The queues used to communicate between tasks and ISR's. */
static QueueHandle_t xRxedChars;
static portBASE_TYPE xTxHasEnded;

/******************************************************************************/
/* Communication Functions                                                    */
/******************************************************************************/

void xSerialPortDMAInit(void) {
    DMA1CONbits.CHEN = 0;
    DMA1CONbits.SIZE = 1;
    DMA1CONbits.DIR = 1;
    DMA1CONbits.HALF = 0;
    DMA1CONbits.NULLW = 0;
    DMA1CONbits.AMODE = 0;
    DMA1CONbits.MODE = 1;

    DMA1REQ = 0x000c; // Select UART1 Transmitter

    DMA1STA = __builtin_dmaoffset(BufferTx);
    DMA1PAD = (volatile unsigned int) &U1TXREG;
    
    /* It is assumed that this function is called prior to the scheduler being
    started.  Therefore interrupts must not be allowed to occur yet as they
    may attempt to perform a context switch. */
    portDISABLE_INTERRUPTS();

    IPC3bits.DMA1IP = configKERNEL_INTERRUPT_PRIORITY; // Set DMA Interrupt Priority Level
    IFS0bits.DMA1IF = serCLEAR_FLAG; // Clear DMA Interrupt Flag
    IEC0bits.DMA1IE = serINTERRUPT_ENABLE; // Enable DMA interrupt
}

xComPortHandle xSerialPortInitMinimal(unsigned long ulWantedBaud, unsigned portBASE_TYPE uxQueueLength) {
    char cChar;

    /* Create the queues used by the com test task. */
    xRxedChars = xQueueCreate(uxQueueLength, (unsigned portBASE_TYPE) sizeof ( signed char));

    /* Setup the UART. */
    U1MODEbits.BRGH = serLOW_SPEED;
    U1MODEbits.STSEL = serONE_STOP_BIT;
    U1MODEbits.PDSEL = serEIGHT_DATA_BITS_NO_PARITY;
    U1MODEbits.ABAUD = serAUTO_BAUD_OFF;
    U1MODEbits.LPBACK = serLOOPBACK_OFF;
    U1MODEbits.WAKE = serWAKE_UP_DISABLE;
    U1MODEbits.UEN = serNO_HARDWARE_FLOW_CONTROL;
    U1MODEbits.IREN = serNO_IRDA;
    U1MODEbits.USIDL = serCONTINUE_IN_IDLE_MODE;
    U1MODEbits.UARTEN = serUART_ENABLED;

    U1BRG = (unsigned short) (((float) configCPU_CLOCK_HZ / ((float) 16 * (float) ulWantedBaud)) - (float) 0.5);

    U1STAbits.URXISEL = serINTERRUPT_ON_SINGLE_CHAR;
    U1STAbits.UTXEN = serTX_ENABLE;
    U1STAbits.UTXINV = serNORMAL_IDLE_STATE;
    U1STAbits.UTXISEL0 = serINTERRUPT_ON_SINGLE_CHAR;
    U1STAbits.UTXISEL1 = serINTERRUPT_ON_SINGLE_CHAR;

    // Initialization TxDMA
    xSerialPortDMAInit();
    
    /* It is assumed that this function is called prior to the scheduler being
    started.  Therefore interrupts must not be allowed to occur yet as they
    may attempt to perform a context switch. */
    portDISABLE_INTERRUPTS();

    IFS0bits.U1RXIF = serCLEAR_FLAG;
    IFS0bits.U1TXIF = serCLEAR_FLAG;
    IPC2bits.U1RXIP = configKERNEL_INTERRUPT_PRIORITY;
    IPC3bits.U1TXIP = configKERNEL_INTERRUPT_PRIORITY;
    IEC0bits.U1TXIE = serINTERRUPT_DISABLE;
    IEC0bits.U1RXIE = serINTERRUPT_ENABLE;
    
    /* Clear the Rx buffer. */
    while (U1STAbits.URXDA == serSET_FLAG) {
        cChar = U1RXREG;
    }

    xTxHasEnded = pdTRUE;

    return NULL;
}

signed portBASE_TYPE xSerialPutBuff(xComPortHandle pxPort, signed char* buff, size_t size) {
    /* Only one port is supported. */
    (void) pxPort;

    //Wait to complete send packet from UART1 and DMA1.
    while ((U1STAbits.TRMT == 0) && (DMA1CONbits.CHEN == 0));
    
    // Enter in critical section
    portENTER_CRITICAL();
    //Copy the message on Buffer DMA TX
    memcpy(&BufferTx, buff, size);
    // Initialize the DMA controller
    DMA1CNT = size - 1; // # of DMA requests
    // Exit from critical section
    portEXIT_CRITICAL();
    
    DMA1CONbits.CHEN = 1; // Enable DMA1 Channel
    DMA1REQbits.FORCE = 1; // Manual mode: Kick-start the 1st transfer
    
    return pdPASS;
}
/**
 * When receive the DMA interrupt unlock the UART writer
 */
void __attribute__((interrupt, no_auto_psv)) _DMA1Interrupt(void) {
    
    IFS0bits.DMA1IF = 0; // Clear the DMA1 Interrupt Flag
}

signed portBASE_TYPE xSerialGetChar(xComPortHandle pxPort, signed char *pcRxedChar, TickType_t xBlockTime) {
    /* Only one port is supported. */
    (void) pxPort;

    /* Get the next character from the buffer.  Return false if no characters
    are available or arrive before xBlockTime expires. */
    if (xQueueReceive(xRxedChars, pcRxedChar, xBlockTime)) {
        return pdTRUE;
    } else {
        return pdFALSE;
    }
}

void __attribute__((__interrupt__, auto_psv)) _U1RXInterrupt(void) {
    char cChar;
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    /* Get the character and post it on the queue of Rxed characters.
    If the post causes a task to wake force a context switch as the woken task
    may have a higher priority than the task we have interrupted. */
    IFS0bits.U1RXIF = serCLEAR_FLAG;
    while (U1STAbits.URXDA) {
        cChar = U1RXREG;
        xQueueSendFromISR(xRxedChars, &cChar, &xHigherPriorityTaskWoken);
    }

    if (xHigherPriorityTaskWoken != pdFALSE) {
        taskYIELD();
    }
}