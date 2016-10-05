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
#include <peripherals/led.h>

#include "system/peripherals.h"
#include "system/system.h"

#include "motors/motor_control.h"

/*****************************************************************************/
/* Global Variable Declaration                                               */
/*****************************************************************************/
#define DEBUG_ADC

#ifdef DEBUG_ADC
#define ADC_BUFF 16
#else
#define ADC_BUFF 32
#endif

typedef enum _type_conf {
    ADC_SIM_2,
    ADC_SIM_4,
    ADC_SCAN
} type_conf_t;

typedef struct adc_sim_2_channels {
    unsigned int ch0[ADC_BUFF/2];
    unsigned int ch1[ADC_BUFF/2];
} adc_sim_2_channels_t;

typedef struct adc_sim_4_channels {
    unsigned int ch0[ADC_BUFF/4];
    unsigned int ch1[ADC_BUFF/4];
    unsigned int ch2[ADC_BUFF/4];
    unsigned int ch3[ADC_BUFF/4];
} adc_sim_4_channels_t;

typedef union _adc_buffer {
    adc_sim_2_channels_t sim_2_channels;
    adc_sim_4_channels_t sim_4_channels;
    unsigned int buffer[ADC_BUFF];
} adc_buffer_t;

typedef struct _adc_buff_info {
    type_conf_t adc_conf;
    int numadc;
    math_buffer_size_t size_base_2;
    int size;
} adc_buff_info_t;

hardware_bit_t ana_en = REGISTER_INIT(AD1CON1, 15);
hardware_bit_t dma_en = REGISTER_INIT(DMA0CON, 15);

 // ADC buffer, 4 channels (AN0, AN1, AN2, AN3), 32 bytes each, 4 x 32 = 64 bytes
adc_buffer_t AdcBufferA __attribute__((space(dma), aligned(ADC_BUFF*2)));
adc_buffer_t AdcBufferB __attribute__((space(dma), aligned(ADC_BUFF*2)));
adc_buff_info_t info_buffer;

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
    
#ifdef NUM_GPIO
gp_port_def_t portB;
gp_peripheral_t port_B_gpio[NUM_GPIO];
#endif

/*****************************************************************************/
/* User Functions                                                            */
/*****************************************************************************/
/** 
 * Initialization DMA0 for ADC
 */
void InitDMA0(void) {
    // DMA enabled from GPIO library DMA0CONbits.CHEN = 1;
    
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
}
/** 
 * Initialization ADC with read CH0, CH1 simultaneously 
 */
void InitADC_2Sim() {
    // ADC enabled from GPIO library AD1CON1bits.ADON = 1;
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
#ifdef DEBUG_ADC
    AD1CON4bits.DMABL = 0b011;
#else
    // ADC_BUFF = 32 -> ADC_BUFF/2 = 16
    AD1CON4bits.DMABL = 0b100; // Allocates 16 words of buffer to each analog input
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
}
/**
 * Initialization ADC with read CH0, CH1, CH2, CH3 simultaneously 
 */
void InitADC_4Sim() {
    // ADC enabled from GPIO library AD1CON1bits.ADON = 1;
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
#ifdef DEBUG_ADC
    AD1CON4bits.DMABL = 0b010;
#else    
    // ADC_BUFF = 32 -> ADC_BUFF/4 = 8
    AD1CON4bits.DMABL = 0b011; // Allocates 8 words of buffer to each analog input
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
}
/**
 * Callback to setup ADC from GPIO library. It's required from GPIO library
 * @return return if ADC are correctly configured
 */
bool adc_config(void) {
    IFS0bits.AD1IF = 0; // Clear the A/D interrupt flag bit
    IFS0bits.DMA0IF = 0; // Clear DMA Interrupt Flag
    //Evaluate the number of analog pin on AD1PCFGL
    info_buffer.numadc = NumberOfSetBits((~AD1PCFGL) << 7);
    switch(info_buffer.numadc){
        case 2:
            info_buffer.adc_conf = ADC_SIM_2;
#ifdef DEBUG_ADC
            info_buffer.size_base_2 = MATH_BUFF_8;
#else
            info_buffer.size_base_2 = MATH_BUFF_16;
#endif      
            info_buffer.size = ADC_BUFF/2;
            InitADC_2Sim();
            break;
        case 4:
            info_buffer.adc_conf = ADC_SIM_4;
#ifdef DEBUG_ADC
            info_buffer.size_base_2 = MATH_BUFF_4;
#else
            info_buffer.size_base_2 = MATH_BUFF_8;
#endif
            info_buffer.size = ADC_BUFF/4;
            InitADC_4Sim();
            break;
        default:
            info_buffer.adc_conf = ADC_SCAN;
            //TODO
            return false;
            break;
    }
    return true;
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

//    /* Setup port direction */
//    // weak pullups enable
//    CNPU1 = 0xffff;
//    CNPU2 = 0x9fff; // Pull up on CN29 and CN30 must not be enable to avoid problems with clock!!! by Walt
//    Removed for ADC ... now Works better without this pullups enable.
//    REMOVE after CHECK the code for RoboController and Motion Control

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
    
    InitDMA0();   ///< Open DMA0 for buffering measures ADC
    
#ifdef NUM_GPIO
    // When initialized AD1PCFGL = 0xFFFF set all Analog ports as digital
    gpio_init(&ana_en, &dma_en, &AD1PCFGL, &adc_config, 2, &portA, &portB);
#endif
    
    TRISCbits.TRISC3 = 0;
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
    hEvent_t event_led = LED_Init(1000, &led_controller[0], LED_NUM);
    // Register event LED
    register_time(SYSTEM_EVENT_LED,event_led);
}

inline void UpdateBlink(short num, short blink) {
    LED_updateBlink(led_controller, num, blink);
}

unsigned int current[NUM_MOTORS];
unsigned int voltage[NUM_MOTORS];

inline void ProcessADCSamples(adc_buffer_t* AdcBuffer) {
    unsigned int t = TMR1; // Timing function
    //static int i, counter, adc;
    
    switch(info_buffer.adc_conf) {
        case ADC_SIM_2:
            // Shift the value to cover all int range
            // TODO use a builtin
            gpio_ProcessADCSamples(1, (statistic_buff_mean(AdcBuffer->sim_2_channels.ch0, 0, info_buffer.size_base_2)));
            gpio_ProcessADCSamples(0, (statistic_buff_mean(AdcBuffer->sim_2_channels.ch1, 0, info_buffer.size_base_2)));
            break;
        case ADC_SIM_4:
            // Shift the value to cover all int range
            // TODO use a builtin
            current[MOTOR_ZERO] = statistic_buff_mean(AdcBuffer->sim_4_channels.ch0, 0, info_buffer.size_base_2);
            voltage[MOTOR_ZERO] = statistic_buff_mean(AdcBuffer->sim_4_channels.ch1, 0, info_buffer.size_base_2);
            current[MOTOR_ONE] = statistic_buff_mean(AdcBuffer->sim_4_channels.ch2, 0, info_buffer.size_base_2);
            voltage[MOTOR_ONE] = statistic_buff_mean(AdcBuffer->sim_4_channels.ch3, 0, info_buffer.size_base_2);
#ifndef INTERNAL_CONTROL
            gpio_ProcessADCSamples(0, current[MOTOR_ZERO]);
            gpio_ProcessADCSamples(1, voltage[MOTOR_ZERO]);
            gpio_ProcessADCSamples(2, current[MOTOR_ONE]);
            gpio_ProcessADCSamples(3, voltage[MOTOR_ONE]);
#endif
//            gpio_ProcessADCSamples(0, (statistic_buff_mean(AdcBuffer->sim_4_channels.ch0, 0, info_buffer.size_base_2)));
//            gpio_ProcessADCSamples(1, (statistic_buff_mean(AdcBuffer->sim_4_channels.ch1, 0, info_buffer.size_base_2)));
//            gpio_ProcessADCSamples(2, (statistic_buff_mean(AdcBuffer->sim_4_channels.ch2, 0, info_buffer.size_base_2)));
//            gpio_ProcessADCSamples(3, (statistic_buff_mean(AdcBuffer->sim_4_channels.ch3, 0, info_buffer.size_base_2)));
            break;
        case ADC_SCAN:
//            counter = 0;
//            adc = (~AD1PCFGL & 0b0000000111111111);
//            for(i = 0; counter < info_buffer.numadc; ++i) {
//                if(REGISTER_MASK_READ(&adc, BIT_MASK(i))) {
//                    gpio_ProcessADCSamples(i, statistic_buff_mean(AdcBuffer->buffer, counter*info_buffer.size, info_buffer.size_base_2));
//                    counter++;
//                }
//            }
            break;
    }
    
#ifdef INTERNAL_CONTROL
    // Launch the motor current control for motor zero
    CurrentControl(MOTOR_ZERO, current[MOTOR_ZERO], voltage[MOTOR_ZERO]);
    // Launch the motor current control for motor one
    CurrentControl(MOTOR_ONE, current[MOTOR_ONE], voltage[MOTOR_ONE]);
#endif    
    update_adc_time(t, TMR1);
}

void __attribute__((interrupt, auto_psv)) _DMA0Interrupt(void) {
    static unsigned short DmaBuffer = 0;
    if(DmaBuffer == 0) {
        ProcessADCSamples(&AdcBufferA);
    } else {
        ProcessADCSamples(&AdcBufferB);
    }
    LATCbits.LATC3 ^= 1;
    DmaBuffer ^= 1;
    IFS0bits.DMA0IF = 0; // Clear the DMA0 Interrupt Flag
}
