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

#include "peripherals/adc.h"

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

// ADC buffer, 4 channels (AN0, AN1, AN2, AN3), 32 bytes each, 4 x 32 = 64 bytes
unsigned int AdcBufferA[ADC_BUFF] __attribute__((space(dma), aligned(ADC_BUFF*2)));
unsigned int AdcBufferB[ADC_BUFF] __attribute__((space(dma), aligned(ADC_BUFF*2)));

/******************************************************************************/
/* System Level Functions                                                     */
/******************************************************************************/
/** 
 * Initialization DMA0 for ADC
 */
void InitDMA0(void) {
    DMA0REQ = 13; // Select ADC1 as DMA Request source

    DMA0CONbits.AMODE = 2;
    DMA0CONbits.MODE = 2;

    DMA0STA = __builtin_dmaoffset(&AdcBufferA);
    DMA0STB = __builtin_dmaoffset(&AdcBufferB);
    DMA0PAD = (volatile unsigned int) &ADC1BUF0; // Point DMA to ADC1BUF0

    IFS0bits.DMA0IF = 0; // Clear DMA Interrupt Flag
    IPC1bits.DMA0IP = 1; // Set DMA Interrupt Priority Level
    IEC0bits.DMA0IE = 1; // Enable DMA interrupt
    
    DMA0CNT = ADC_BUFF - 1; // DMA request
    
    DMA0CONbits.CHEN = 1;   //< Enable DMA0
}
/** 
 * Initialization ADC with read CH0, CH1 simultaneously 
 */
void InitADC_2Sim(void) {
    // When initialized from GPIO library AD1PCFGL = 0xFFFF set all Analog ports as digital
    AD1CON1bits.FORM = 0;       //< Data Output Format: Integer
    AD1CON1bits.SSRC = 0b111;   //< Sample Clock Source: Internal counter sampling and starts convertions (auto-convert)
    AD1CON1bits.ASAM = 1;       //< ADC Sample Control: Sampling begins immediately after conversion
    AD1CON1bits.AD12B = 0;      //< 10-bit ADC operation
    AD1CON1bits.ADSIDL = 1;     //< stop in idle
    AD1CON1bits.SIMSAM = 1;     //< CH0 CH1 sampled simultaneously
    AD1CON1bits.ADDMABM = 0;    //< DMA buffers are built in scatter/gather mode

    AD1CON2bits.CSCNA = 0;      //< Input scan: Do not scan inputs
    AD1CON2bits.CHPS = 0b01;    //< Convert CH0 and CH1
    AD1CON2bits.BUFM = 0;       //< filling buffer from start address
    AD1CON2bits.ALTS = 0;       //< ONLY sample A
    AD1CON2bits.SMPI = 0b0000;  //< number of DMA buffers -1
    
    AD1CON3bits.ADRC = 0;       //< ADC Clock is derived from Systems Clock
    AD1CON3bits.SAMC = 2;		// Auto Sample Time = 2*Tad		
    AD1CON3bits.ADCS = 2;		// ADC Conversion Clock Tad=Tcy*(ADCS+1)= (1/40M)*3 = 75ns (13.3Mhz)
                                // ADC Conversion Time for 10-bit Tc=12*Tad =  900ns (1.1MHz)
#ifdef ADC_HIGH_FREQ
    AD1CON4bits.DMABL = 0b010;
#else
    // ADC_BUFF = 16 -> ADC_BUFF/2 = 8
    AD1CON4bits.DMABL = 0b011; // Allocates 16 words of buffer to each analog input
#endif
    
    AD1CHS0bits.CH0SA = 1;      //< CH0 pos -> AN1
    AD1CHS0bits.CH0NA = 0;      //< CH0 neg -> Vrefl
    AD1CHS0bits.CH0SB = 0;      //< don't care -> sample B
    AD1CHS0bits.CH0NB = 0;      //< don't care -> sample B
    
    AD1CHS123bits.CH123SA = 0;  //< CH1 = AN0, CH2=AN1, CH3=AN2
    AD1CHS123bits.CH123NA = 0;  //< CH1,2,3 negative input = Vrefl
    AD1CHS123bits.CH123SB = 0;  //< don't care -> sample B
    AD1CHS123bits.CH123NB = 0;  //< don't care -> sample B

    IFS0bits.AD1IF = 0; // Clear the A/D interrupt flag bit
    IEC0bits.AD1IE = 0; // Do Not Enable A/D interrupt
    
    AD1CSSL = 0;
    
    AD1CON1bits.ADON = 0;       //< Disable ADC
}
/**
 * Initialization ADC with read CH0, CH1, CH2, CH3 simultaneously 
 */
void InitADC_4Sim(void) {
    // When initialized from GPIO library AD1PCFGL = 0xFFFF set all Analog ports as digital
    AD1CON1bits.FORM = 0;       //< Data Output Format: Integer
    AD1CON1bits.SSRC = 0b111;   //< Sample Clock Source: Internal counter sampling and starts convertions (auto-convert)
    AD1CON1bits.ASAM = 1;       //< ADC Sample Control: Sampling begins immediately after conversion
    AD1CON1bits.AD12B = 0;      //< 10-bit ADC operation
    AD1CON1bits.ADSIDL = 1;     //< stop in idle
    AD1CON1bits.SIMSAM = 1;     //< CH0, CH1, CH2 and CH3 sampled simultaneously
    AD1CON1bits.ADDMABM = 0;    //< DMA buffers are built in scatter/gather mode
    
    AD1CON2bits.CSCNA = 0;      //< Input scan: Do not scan inputs
    AD1CON2bits.CHPS = 0b11;    //< Convert CH0, CH1, CH2 and CH3
    AD1CON2bits.BUFM = 0;       //< filling buffer from start address
    AD1CON2bits.ALTS = 0;       //< ONLY sample A
    AD1CON2bits.SMPI = 0b0000;  //< number of DMA buffers -1
    
    AD1CON3bits.ADRC = 0;       //< ADC Clock is derived from Systems Clock
    AD1CON3bits.SAMC = 2;		// Auto Sample Time = 2*Tad		
    AD1CON3bits.ADCS = 2;		// ADC Conversion Clock Tad=Tcy*(ADCS+1)= (1/40M)*3 = 75ns (13.3Mhz)
                                // ADC Conversion Time for 10-bit Tc=12*Tad =  900ns (1.1MHz)
#ifdef ADC_HIGH_FREQ
    AD1CON4bits.DMABL = 0b001;
#else    
    // ADC_BUFF = 16 -> ADC_BUFF/4 = 4
    AD1CON4bits.DMABL = 0b010; // Allocates 8 words of buffer to each analog input
#endif
    AD1CHS0bits.CH0SA = 3;      //< CH0 pos -> AN3
    AD1CHS0bits.CH0NA = 0;      //< CH0 neg -> Vrefl
    AD1CHS0bits.CH0SB = 0;      //< don't care -> sample B
    AD1CHS0bits.CH0NB = 0;      //< don't care -> sample B
    
    AD1CHS123bits.CH123SA = 0;  //< CH1 = AN0, CH2=AN1, CH3=AN2
    AD1CHS123bits.CH123NA = 0;  //< CH1,2,3 negative input = Vrefl
    AD1CHS123bits.CH123SB = 0;  //< don't care -> sample B
    AD1CHS123bits.CH123NB = 0;  //< don't care -> sample B
    
    IFS0bits.AD1IF = 0; // Clear the A/D interrupt flag bit
    IEC0bits.AD1IE = 0; // Do Not Enable A/D interrupt
    
    AD1CSSL = 0;
    
    AD1CON1bits.ADON = 0;       //< Disable ADC
}

void __attribute__((interrupt, auto_psv)) _DMA0Interrupt(void) {
    IFS0bits.DMA0IF = 0;    // Clear the DMA0 Interrupt Flag
    
    static unsigned short DmaBuffer = 0;
    if(DmaBuffer == 0) {
        ADC_controller(&AdcBufferA[0]);
    } else {
        ADC_controller(&AdcBufferB[0]);
    }
    // Change buffer to read
    DmaBuffer ^= 1;
    // ISR exit
    taskYIELD();
}