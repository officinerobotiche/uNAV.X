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

#include <or_math/math.h>
#include <peripherals/led.h>

#include "system/peripherals.h"
#include "system/system.h"

#include "motors/motor_control.h"

/*****************************************************************************/
/* Global Variable Declaration                                               */
/*****************************************************************************/

#define ADC_CHANNELS 4
#define ADC_BUFF 64
#define TOT_ADC_BUFF ADC_CHANNELS * ADC_BUFF

typedef enum _type_conf {
    ADC_SIM_2,
    ADC_SIM_4,
    ADC_SCAN
} type_conf_t;

typedef struct _adc_channels {
    unsigned int ch0[ADC_BUFF];
    unsigned int ch1[ADC_BUFF];
    unsigned int ch2[ADC_BUFF];
    unsigned int ch3[ADC_BUFF];
} adc_channels_t;

typedef union _adc_buffer {
    adc_channels_t channels;
    unsigned int buffer[TOT_ADC_BUFF];
} adc_buffer_t;

 // ADC buffer, 4 channels (AN0, AN1, AN2, AN3), 32 bytes each, 4 x 32 = 64 bytes
adc_buffer_t AdcBufferA __attribute__((space(dma), aligned(TOT_ADC_BUFF)));
adc_buffer_t AdcBufferB __attribute__((space(dma), aligned(TOT_ADC_BUFF)));
type_conf_t adc_conf = ADC_SCAN;

#ifdef UNAV_V1
/// Number of available LEDs
#define LED_NUM 4
/// Number of available GPIOs
#define NUM_GPIO 9
#elif ROBOCONTROLLER_V3
/// Number of available LEDs
#define LED_NUM 2
/// Number of available GPIOs
#define NUM_GPIO 2
#elif MOTION_CONTROL
/// Number of available LEDs
#define LED_NUM 1
#endif

gp_analog_t adc_gpio_data[7];
gp_port_def_t portA;
gp_peripheral_t port_A_gpio[4];
led_control_t led_controller[LED_NUM];
int numadc = 0;
    
#ifdef NUM_GPIO
gp_port_def_t portB;
gp_peripheral_t port_B_gpio[NUM_GPIO];
#endif

/*****************************************************************************/
/* User Functions                                                            */
/*****************************************************************************/
/** 
 * Initialization ADC for measure current motors
 */
void InitADC(void) {
    AD1CON1bits.FORM = 0; // Data Output Format: Integer
    AD1CON1bits.SSRC = 0b111; // Sample Clock Source: Internal counter sampling and starts convertions (auto-convert)
    AD1CON1bits.ASAM = 1; // ADC Sample Control: Sampling begins immediately after conversion
    AD1CON1bits.AD12B = 0; // 10-bit ADC operation
    AD1CON1bits.ADSIDL = 1; // stop in idle
    AD1CON1bits.SIMSAM = 1; // CH0 CH1 sampled simultaneously

    AD1CON2bits.CSCNA = 0; // Input scan: Do not scan inputs
    AD1CON2bits.CHPS = 1; // Convert CH0 and CH1
    AD1CON2bits.BUFM = 0; // filling buffer from start address
    AD1CON2bits.ALTS = 0; // sample A

    AD1CON3bits.ADRC = 0; // ADC Clock is derived from Systems Clock
    AD1CON3bits.SAMC = 0b11111; // 31 Tad auto sample time
    AD1CON3bits.ADCS = ADC_BUFF - 1; // ADC Conversion Clock Tad=Tcy*(ADCS+1)= (1/40M)*64 = 1.6us (625Khz)
    // ADC Conversion Time for 10-bit Tc=12*Tab = 19.2us

    AD1CON1bits.ADDMABM = 0; // DMA buffers are built in scatter/gather mode
    AD1CON2bits.SMPI = 0b0001; // number of DMA buffers -1
    AD1CON4bits.DMABL = 0b110; // 64 word DMA buffer for each analog input

    AD1CHS123bits.CH123NB = 0; // don't care -> sample B
    AD1CHS123bits.CH123SB = 0; // don't care -> sample B
    AD1CHS123bits.CH123NA = 0; // CH1,2,3 negative input = Vrefl
    AD1CHS123bits.CH123SA = 0; // CH1 = AN0, CH2=AN1, CH3=AN2

    AD1CHS0bits.CH0NB = 0; // don't care -> sample B
    AD1CHS0bits.CH0SB = 0; // don't care -> sample B
    AD1CHS0bits.CH0NA = 0; // CH0 neg -> Vrefl
    AD1CHS0bits.CH0SA = 1; // CH0 pos -> AN1

    AD1PCFGL = 0xFFFF; // set all Analog ports as digital

    IFS0bits.AD1IF = 0; // Clear the A/D interrupt flag bit
    IEC0bits.AD1IE = 0; // Do Not Enable A/D interrupt
}
/** 
 * Initialization DMA0 for ADC current
 */
void InitDMA0(void) {
    DMA0REQ = 13; // Select ADC1 as DMA Request source

    DMA0CONbits.AMODE = 2; // Peripheral Indirect Addressing mode
    DMA0CONbits.MODE = 2; // Ping pong

    DMA0STA = __builtin_dmaoffset(&AdcBufferA);
    DMA0STB = __builtin_dmaoffset(&AdcBufferB);
    DMA0PAD = (volatile unsigned int) &ADC1BUF0; // Point DMA to ADC1BUF0

    IFS0bits.DMA0IF = 0; // Clear DMA Interrupt Flag
    IPC1bits.DMA0IP = ADC_DMA_LEVEL; // Set DMA Interrupt Priority Level
    IEC0bits.DMA0IE = 1; // Enable DMA interrupt
}

bool adc_config(void) {
    bool state = false;
    /// Get all ADC configured
    AD1CON1bits.ADON = 0; // module off
    DMA0CONbits.CHEN = 0; // Disable DMA
    IFS0bits.DMA0IF = 0; // Clear DMA Interrupt Flag
    if(AD1PCFGL == 0b0000000111111100) {
        numadc = 2;
        adc_conf = ADC_SIM_2;
        AD1CON1bits.SIMSAM = 1; // CH0 CH1 sampled simultaneously
        AD1CON2bits.CSCNA = 0; // Input scan: Do not scan inputs
        AD1CON2bits.CHPS = 1; // Convert CH0 and CH1
        AD1CON2bits.SMPI = 1; // number of DMA buffers -1
        AD1CHS0bits.CH0SA = 1; // CH0 pos -> AN1
        /// DMA Configuration
        DMA0CNT = 2 * ADC_BUFF - 1; // 64 DMA request
        
        /// Complete configuration
        state = true;
    } else if(AD1PCFGL == 0b0000000111110000) {
        numadc = 4;
        adc_conf = ADC_SIM_4;
        AD1CON1bits.SIMSAM = 1; // CH0 CH1 sampled simultaneously
        AD1CON2bits.CSCNA = 0; // Input scan: Do not scan inputs
        AD1CON2bits.CHPS = 0b11; // Convert CH0, CH1, CH2 and CH3
        AD1CON2bits.SMPI = 3; // number of DMA buffers -1
        AD1CHS0bits.CH0SA = 3; // CH0 pos -> AN3
        /// DMA Configuration
        DMA0CNT = 4 * ADC_BUFF - 1; // 64 DMA request
        /// Complete configuration
        state = true;
    } else if(AD1PCFGL != 0b0000000111111111) {
        numadc = NumberOfSetBits((int) (~AD1PCFGL & 0b0000000111111111));
        adc_conf = ADC_SCAN;
        AD1CON1bits.SIMSAM = 0; // CH0 sampled
        AD1CON2bits.CSCNA = 1; // Input scan: Do not scan inputs
        AD1CON2bits.CHPS = 0; // Convert CH0
        AD1CON2bits.SMPI = numadc - 1; // number of DMA buffers -1
        AD1CHS0bits.CH0SA = 0; // CH0 pos -> AN0
        /// Setup scanning mode
        AD1CSSL = AD1PCFGL;
        /// DMA Configuration
        DMA0CNT = (TOT_ADC_BUFF / numadc) - 1;
        /// Complete configuration
        state = true;
    }
    //Enable or disable the module
    if(numadc > 0 && state == true) {
        AD1CON1bits.ADON = 1; // module on
        DMA0CONbits.CHEN = 1; // Enable DMA
    }
    return state;
}

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

    /* Setup port direction */
    // weak pullups enable
    CNPU1 = 0xffff;
    CNPU2 = 0x9fff; // Pull up on CN29 and CN30 must not be enable to avoid problems with clock!!! by Walt

    GPIO_PORT_INIT(portA, &port_A_gpio[0], 4);
    /// CURRENT 1
    GPIO_ANALOG_CONF(adc_gpio_data[0], 0);
    GPIO_INIT_ANALOG(port_A_gpio[0], A, 0, &adc_gpio_data[0]);
    GPIO_ANALOG_CONF(adc_gpio_data[1], 1);
    GPIO_INIT_ANALOG(port_A_gpio[1], A, 1, &adc_gpio_data[1]);
    GPIO_ANALOG_CONF(adc_gpio_data[2], 2);
    GPIO_INIT_ANALOG(port_A_gpio[2], B, 0, &adc_gpio_data[2]);
    GPIO_ANALOG_CONF(adc_gpio_data[3], 3);
    GPIO_INIT_ANALOG(port_A_gpio[3], B, 1, &adc_gpio_data[3]);
#ifdef UNAV_V1
    GPIO_PORT_INIT(portB, &port_B_gpio[0], NUM_GPIO);
    // GPIO
    GPIO_INIT(port_B_gpio[0], A, 9); // GP0 - HALT //< TO BE DEFINE
    GPIO_ANALOG_CONF(adc_gpio_data[4], 6);
    GPIO_INIT_ANALOG(port_B_gpio[1], C, 0, &adc_gpio_data[4]); // GP1
    GPIO_ANALOG_CONF(adc_gpio_data[5], 7);
    GPIO_INIT_ANALOG(port_B_gpio[2], C, 1, &adc_gpio_data[5]); // GP2
    GPIO_ANALOG_CONF(adc_gpio_data[6], 8);
    GPIO_INIT_ANALOG(port_B_gpio[3], C, 2, &adc_gpio_data[6]); // GP3
    GPIO_INIT(port_B_gpio[4], C, 3); // GP4
    GPIO_INIT(port_B_gpio[5], A, 4); // GP5
    GPIO_INIT(port_B_gpio[6], B, 4); // GP6
    GPIO_INIT(port_B_gpio[7], B, 7); // GP7
    GPIO_INIT(port_B_gpio[8], A, 8); // GP8
#elif ROBOCONTROLLER_V3
    // GPIO
    GPIO_INIT(port_B_gpio[0], A, 7); // GP0
    GPIO_INIT(port_B_gpio[1], A, 10);// GP1
    GPIO_INIT(port_B_gpio[2], B, 4); // GP2
    GPIO_INIT(port_B_gpio[3], C, 2); // GP3
    GPIO_INIT(port_B_gpio[4], C, 3); // GP4
    GPIO_INIT(port_B_gpio[5], B, 7); // GP5
#elif MOTION_CONTROL
    _TRISB5 = 1;
    _TRISB6 = 1;
    _TRISB10 = 1;
    _TRISB11 = 1;
#else
#error Configuration error. Does not selected a board!
#endif
    
    InitADC();    ///< Open ADC for measure current motors
    InitDMA0();   ///< Open DMA0 for buffering measures ADC
    
#ifdef NUM_GPIO
    gpio_init(&AD1PCFGL, &adc_config, 2, &portA, &portB);
#endif
}

void InitLEDs(void) {
#ifdef UNAV_V1
    GPIO_INIT_TYPE(led_controller[0].gpio, C, 6, GPIO_OUTPUT);
    GPIO_INIT_TYPE(led_controller[1].gpio, C, 7, GPIO_OUTPUT);
    GPIO_INIT_TYPE(led_controller[2].gpio, C, 8, GPIO_OUTPUT);
    GPIO_INIT_TYPE(led_controller[3].gpio, C, 9, GPIO_OUTPUT);
#elif ROBOCONTROLLER_V3
    GPIO_INIT_TYPE(led_controller[0].gpio, A, 8, GPIO_OUTPUT);
    GPIO_INIT_TYPE(led_controller[1].gpio, A, 9, GPIO_OUTPUT);
#elif MOTION_CONTROL
    GPIO_INIT_TYPE(led_controller[0].gpio, A, 4, GPIO_OUTPUT);
#endif
    LED_Init(get_system_parameters().FREQ_SYSTEM, &led_controller[0], LED_NUM);
}

inline void UpdateBlink(short num, short blink) {
    LED_updateBlink(led_controller, num, blink);
}

inline void ProcessADCSamples(adc_buffer_t* AdcBuffer) {
    static int i, counter;
    switch(adc_conf) {
        case ADC_SIM_2:
            gpio_ProcessADCSamples(0, AdcBuffer->channels.ch0, ADC_BUFF);
            gpio_ProcessADCSamples(1, AdcBuffer->channels.ch1, ADC_BUFF);
            break;
        case ADC_SIM_4:
            gpio_ProcessADCSamples(3, AdcBuffer->channels.ch0, ADC_BUFF);
            gpio_ProcessADCSamples(0, AdcBuffer->channels.ch1, ADC_BUFF);
            gpio_ProcessADCSamples(1, AdcBuffer->channels.ch2, ADC_BUFF);
            gpio_ProcessADCSamples(2, AdcBuffer->channels.ch3, ADC_BUFF);
            break;
        case ADC_SCAN:
            counter = 0;
            for(i = 0; counter == numadc; ++i) {
                if(REGISTER_MASK_READ(&AD1PCFGL, BIT_MASK(i))) {
                    gpio_ProcessADCSamples_start(i, AdcBuffer->buffer, counter*DMA0CNT, DMA0CNT + 1);
                    counter++;
                }
            }
            break;
    }
}

void __attribute__((interrupt, auto_psv)) _DMA0Interrupt(void) {
    static unsigned short DmaBuffer = 0;
    if(DmaBuffer == 0) {
        ProcessADCSamples(&AdcBufferA);
    } else {
        ProcessADCSamples(&AdcBufferB);
    }
    DmaBuffer ^= 1;
    IFS0bits.DMA0IF = 0; // Clear the DMA0 Interrupt Flag
}
