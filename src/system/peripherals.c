/*
 * Copyright (C) 2015 Officine Robotiche
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

#include <xc.h>                     /* Device header file */

#include <or_math/math.h>
#include <or_math/statistics.h>
#include <or_peripherals/GPIO/led.h>
#include <or_peripherals/GPIO/adc.h>

#include "system/peripherals.h"
#include "system/system.h"

#include "motors/motor_control.h"

// Test pin to check frequency of the code
//#define TEST_PIN // PIN RC3

/*****************************************************************************/
/* Global Variable Declaration                                               */
/*****************************************************************************/

// ADC buffer, 4 channels (AN0, AN1, AN2, AN3), 32 bytes each, 4 x 32 = 64 bytes
unsigned int AdcBufferA[ADC_BUFF] __attribute__((space(dma), aligned(ADC_BUFF*2)));
unsigned int AdcBufferB[ADC_BUFF] __attribute__((space(dma), aligned(ADC_BUFF*2)));

#ifdef UNAV_V1
// Initialization LED on uNAV
LED_t leds[] = {
    GPIO_LED(C, 6), // Led 1 Green
    GPIO_LED(C, 7), // Led 2 Red
    GPIO_LED(C, 8), // Led 3 Yellow
    GPIO_LED(C, 9), // Led 4 Blue
};
// Initialization controllable GPIO
gpio_t portB[] = {
    GPIO_INIT(A, 9, GPIO_OUTPUT),   // GP0
    GPIO_INIT(C, 0, GPIO_OUTPUT),   // GP1
    GPIO_INIT(C, 1, GPIO_OUTPUT),   // GP2
    GPIO_INIT(C, 2, GPIO_OUTPUT),   // GP3
    GPIO_INIT(C, 3, GPIO_OUTPUT),   // GP4
    GPIO_INIT(A, 4, GPIO_OUTPUT),   // GP5
    GPIO_INIT(B, 4, GPIO_OUTPUT),   // GP6
    GPIO_INIT(B, 7, GPIO_OUTPUT),   // GP7
    GPIO_INIT(A, 8, GPIO_OUTPUT),   // GP8
};
/// Number of available GPIOs
#define NUM_GPIO (sizeof(portB) / ( sizeof(portB[0])))
#elif ROBOCONTROLLER_V3
// Initialization LED on RoboController
LED_t leds[] = {
    GPIO_LED(A, 8), // Led 1 green
    GPIO_LED(A, 9), // Led 2 green
};
// Initialization controllable GPIO
gpio_t portB[] = {
    GPIO_INIT(A, 7, GPIO_OUTPUT),   // GP0
    GPIO_INIT(A, 10, GPIO_OUTPUT),  // GP1
    GPIO_INIT(B, 4, GPIO_OUTPUT),   // GP2
    GPIO_INIT(C, 2, GPIO_OUTPUT),   // GP3
    GPIO_INIT(C, 3, GPIO_OUTPUT),   // GP4
    GPIO_INIT(B, 7, GPIO_OUTPUT),   // GP5
};
/// Number of available GPIOs
#define NUM_GPIO (sizeof(portB) / ( sizeof(portB[0])))
#elif MOTION_CONTROL
// Initialization LED on motion control
LED_t leds[] = {
    GPIO_LED(A, 4), // Led Blue
};
gpio_t portB[] = {};
/// Number of available GPIOs
#define NUM_GPIO 0
#endif
/// Number of available LEDs
#define LED_NUM (sizeof(leds) / ( sizeof(leds[0])))
// Initialization LED controller
LED_controller_t LED_CONTROLLER = LED_CONTROLLER(leds, LED_NUM, 1000);

/*****************************************************************************/
/* User Functions                                                            */
/*****************************************************************************/
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
    IPC1bits.DMA0IP = ADC_DMA_LEVEL; // Set DMA Interrupt Priority Level
    IEC0bits.DMA0IE = 1; // Enable DMA interrupt
    
    DMA0CNT = ADC_BUFF - 1; // DMA request
    
    DMA0CONbits.CHEN = 1;   //< Enable DMA0
}
/** 
 * Initialization ADC with read CH0, CH1 simultaneously 
 */
void InitADC_2Sim() {
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
    
    AD1CON1bits.ADON = 1;       //< Enable ADC
}
/**
 * Initialization ADC with read CH0, CH1, CH2, CH3 simultaneously 
 */
void InitADC_4Sim() {
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
    
    AD1CON1bits.ADON = 1;       //< Enable ADC
}
/**
 * Initialization remappable peripherals and setup GPIO library
 */
void Peripherals_Init(void) {
    
    // Peripheral PIN remapping
    //*************************************************************
    // Unlock Registers
    //*************************************************************
    asm volatile ( "mov #OSCCONL, w1 \n"
                "mov #0x45, w2 \n"
                "mov #0x57, w3 \n"
                "mov.b w2, [w1] \n"
                "mov.b w3, [w1] \n"
                "bclr OSCCON, #6 ");
#ifdef UNAV_V1
    // Input capture
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
       
#elif ROBOCONTROLLER_V3
    // Input capture
    RPINR7bits.IC1R = 22; // IC1 To Pin RP22
    RPINR7bits.IC2R = 24; // IC2 To Pin RP24
    // QEI
    RPINR14bits.QEA1R = 22; // QEA1 To Pin RP22
    RPINR14bits.QEB1R = 23; // QEB1 To Pin RP23
    RPINR16bits.QEA2R = 24; // QEA2 To Pin RP24
    RPINR16bits.QEB2R = 25; // QEB2 To Pin RP25
    // UART
    RPINR18bits.U1RXR = 20; // U1RX To Pin RP20
    RPOR10bits.RP21R = 3;   // U1TX To Pin RP21

    //RPINR19bits.U2RXR = 6;  // U2RX To Pin RP6
    RPINR19bits.U2RXR = 8;    // U2RX To Pin RP8
    //RPOR2bits.RP5R = 5;     // U2TX To Pin RP5
    RPOR4bits.RP8R = 5;       // U2TX To Pin RP5
#elif MOTION_CONTROL
    // Input capture
    RPINR7bits.IC1R = 5; // Assign Input Capture 1 To Pin RP5
    RPINR7bits.IC2R = 10; // Assign Input Capture 2 To Pin RP10
    // QEI
    RPINR14bits.QEA1R = 5; // Assign QEA1 To Pin RP5
    RPINR14bits.QEB1R = 6; // Assign QEB1 To Pin RP6
    RPINR16bits.QEA2R = 11; // Assign QEA2 To Pin RP11
    RPINR16bits.QEB2R = 10; // Assign QEB2 To Pin RP10
    //UART RX
    RPINR18bits.U1RXR = 8; // Assign U1RX To Pin RP8
    RPOR4bits.RP9R = 3; // Assign U1Tx To Pin RP9
#else
#error Configuration error. Does not selected a board!
#endif
    //*************************************************************
    // Lock Registers
    //*************************************************************
    asm volatile ( "mov #OSCCONL, w1 \n"
                "mov #0x45, w2 \n"
                "mov #0x57, w3 \n"
                "mov.b w2, [w1] \n"
                "mov.b w3, [w1] \n"
                "bset OSCCON, #6");
    // *********************************** Peripheral PIN selection
    
#if MOTION_CONTROL
    _TRISB5 = 1;
    _TRISB6 = 1;
    _TRISB10 = 1;
    _TRISB11 = 1;
#endif
    
    // Initialize analog port
    // When initialized AD1PCFGL = 0xFFFF set all Analog ports as digital
    gpio_adc_init(&AD1PCFGL);
    InitDMA0();         ///< Open DMA0 for buffering measures ADC
    InitADC_4Sim();     ///< Initialize ADC 4 channels simultaneously
    // Initialize portB
    gpio_init_port(&portB[0], NUM_GPIO);
    // Initialization LED
    LED_Init(&LED_CONTROLLER);
    
    LED_updateBlink(&LED_CONTROLLER, 0, 1);
    LED_updateBlink(&LED_CONTROLLER, 1, 2);
    LED_updateBlink(&LED_CONTROLLER, 2, 3);
    LED_updateBlink(&LED_CONTROLLER, 3, 4);
    
#ifdef TEST_PIN
    TRISCbits.TRISC3 = 0;
#endif
}

inline void UpdateBlink(short num, short blink) {
    LED_updateBlink(&LED_CONTROLLER, num, blink);
}

void __attribute__((interrupt, auto_psv)) _DMA0Interrupt(void) {
    unsigned int t = TMR1; // Timing function
    static unsigned short DmaBuffer = 0;
    if(DmaBuffer == 0) {
        ADC_controller(&AdcBufferA[0]);
    } else {
        ADC_controller(&AdcBufferB[0]);
    }
    // Change buffer to read
    DmaBuffer ^= 1;
    // Update ADC time evaluation
    update_adc_time(t, TMR1);
    
#ifdef TEST_PIN
    __builtin_btg ((unsigned int*)&LATC, 3); //LATCbits.LATC3 ^= 1;
#endif
    IFS0bits.DMA0IF = 0; // Clear the DMA0 Interrupt Flag
}
