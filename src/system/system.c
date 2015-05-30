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

#include <stdint.h>          /* For uint16_t definition                       */
#include <stdbool.h>         /* For true/false definition                     */
#include <pwm12.h>
#include <string.h>

#include <system/events.h>
#include <system/task_manager.h>
#include <system/led.h>

#include "system/system.h"   /* variables/params used by system.c             */

#include "packet/packet.h"
#include "packet/frame_motion.h"
#include "communication/serial.h"
#include "motors/motor_control.h"
#include "high_control/manager.h"

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

unsigned int reset_count = 0;
unsigned char version_date_[] = __DATE__;
unsigned char version_time_[] = __TIME__;
#ifdef UNAV_V1
unsigned char name_board[] = "uNAV";
#elif ROBOCONTROLLER_V3
unsigned char name_board[] = "RoboController";
#elif MOTION_CONTROL
unsigned char name_board[] = "Motion Control";
#endif
#ifdef MOTION_CONTROL
unsigned char author_code[] = "Raffaello Bonghi";
#else
unsigned char author_code[] = "Officine Robotiche";
#endif

unsigned char version_code[] = "v0.5";
unsigned char type_board[] = "Motor Control";
system_parameter_t parameter_system;

// ADC buffer, 2 channels (AN0, AN1), 32 bytes each, 2 x 32 = 64 bytes
extern int AdcBuffer[ADC_CHANNELS][ADC_BUFF] __attribute__((space(dma), aligned(256)));

process_t default_process[NUM_PROCESS_DEFAULT];
process_t motor_process[PROCESS_MOTOR_LENGTH];
process_t motion_process[PROCESS_MOTION_LENGTH];

/******************************************************************************/
/* NEW Global Variable Declaration                                            */
/******************************************************************************/

uint16_t FRQ_CPU = FRTMR1;

#define EVENT_PRIORITY_LOW_ENABLE IEC3bits.RTCIE
#define EVENT_PRIORITY_LOW_FLAG IFS3bits.RTCIF
#define EVENT_PRIORITY_LOW_P IPC15bits.RTCIP
hardware_bit_t RTCIF = {&IFS3, 14};

#define EVENT_PRIORITY_MEDIUM_ENABLE IEC0bits.OC1IE
#define EVENT_PRIORITY_MEDIUM_FLAG IFS0bits.OC1IF
#define EVENT_PRIORITY_MEDIUM_P IPC0bits.OC1IP
hardware_bit_t OC1IF = {&IFS0, 2};

#define EVENT_PRIORITY_HIGH_ENABLE IEC0bits.OC2IE
#define EVENT_PRIORITY_HIGH_FLAG IFS0bits.OC2IF
#define EVENT_PRIORITY_HIGH_P IPC1bits.OC2IP
hardware_bit_t OC2IF = {&IFS0, 6};

//#define MEASURE_ENABLE IEC1bits.OC3IE
//#define MEASURE_FLAG IFS1bits.OC3IF
//#define MEASURE_PRIORITY IPC6bits.OC3IP
//hardware_bit_t OC3IF = {&IFS1, 9};

#ifdef UNAV_V1
/// Number of available LEDs
#define LED_NUM 4
/// LED 1 - Green
hardware_bit_t led_1 = {&LATC, 6};
/// LED 2 - Red
hardware_bit_t led_2 = {&LATC, 7};
/// LED 3 - Yellow
hardware_bit_t led_3 = {&LATC, 8};
/// LED 4 - Blue
hardware_bit_t led_4 = {&LATC, 9};
#elif ROBOCONTROLLER_V3
/// Number of available LEDs
#define LED_NUM 2
/// LED 1 - Green
hardware_bit_t led_1 = {&LATA, 8};
/// LED 2 - Green
hardware_bit_t led_2 = {&LATA, 9};
#elif MOTION_CONTROL
/// Number of available LEDs
#define LED_NUM 1
/// LED 1 - Green
hardware_bit_t led_1 = {&LATA, 4};
#endif

led_control_t led_controller[LED_NUM];

/******************************************************************************/
/* System Level Functions                                                     */
/******************************************************************************/

void ConfigureOscillator(void) {
    PLLFBD = 30; // M=32  //Old configuration: PLLFBD=29 - M=31
    CLKDIVbits.PLLPOST = 0; // N1=2
    CLKDIVbits.PLLPRE = 0; // N2=2
    // Disable Watch Dog Timer
    RCONbits.SWDTEN = 0;
    // Clock switching to incorporate PLL
    // Initiate Clock Switch to Primary
    __builtin_write_OSCCONH(0x03); // Oscillator with PLL (NOSC=0b011)
    __builtin_write_OSCCONL(0x01); // Start clock switching
    while (OSCCONbits.COSC != 0b011); // Wait for Clock switch to occur
    while (OSCCONbits.LOCK != 1) {
    }; // Wait for PLL to lock
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

#ifdef UNAV_V1
    // LED
    _TRISC6 = 0; // LED 1 Green
    _TRISC7 = 0; // LED 2 Green
    _TRISC8 = 0; // LED 3 Yellow
    _TRISC9 = 0; // LED 4 Red
    // Encoders
    _TRISB10 = 1;
    _TRISB11 = 1;
    _TRISB6 = 1;
    _TRISB5 = 1;
    // H bridge
    _TRISA7 = 0; //Enable - Motor 1
    _TRISA10 = 0; //Enable - Motor 2
    _TRISB12 = 0; // PWM1 +
    _TRISB12 = 0; // PWM1 -
    _TRISB12 = 0; // PWM2 +
    _TRISB12 = 0; // PWM2 -
    // GPIO
    _TRISC0 = 1; // GPIO1
    _TRISC1 = 1; // GPIO2
    _TRISC2 = 1; // GPIO3
    _TRISC3 = 1; // GPIO4
    _TRISA4 = 1; // GPIO5
    _TRISB4 = 1; // GPIO6
    _TRISB7 = 1; // GPIO7
    _TRISA8 = 1; // GPIO8
    _TRISA9 = 1; // HALT
    // ADC
    _TRISA0 = 1; // CH1
    _TRISA1 = 1; // CH2
    _TRISB0 = 1; // CH3
    _TRISB1 = 1; // CH4
#elif ROBOCONTROLLER_V3
    // LED
    _TRISA8 = 0; // LED1
    _TRISA9 = 0; // LED2
    // Encodes
    _TRISC6 = 1; // QEA_1
    _TRISC7 = 1; // QEB_1
    _TRISC8 = 1; // QEA_2
    _TRISC9 = 1; // QEB_2
    // H-Bridge
    _TRISA1 = 0; // MOTOR_EN1
    _TRISA4 = 0; // MOTOR_EN2
    // GPIO
    _TRISA7 = 0; // AUX1
    _TRISA10 = 0; // AUX2
    // ADC
    _TRISB2 = 1; // CH1
    _TRISB3 = 1; // CH2
    _TRISC0 = 1; // CH3
    _TRISC1 = 1; // CH4
    // Others
    _TRISB7 = 0; // DIR RS485 UART2
    _TRISB8 = 0; // SDA = Out : Connettore IC2 pin 6
    _TRISB9 = 0; // SCL = Out : Connettore IC2 pin 5
    _TRISB4 = 0; // RB4 = Out : Connettore IC2 pin 4
    _TRISC2 = 0; // OUT Float
    _TRISC3 = 0; // DIR RS485 UART1
#elif MOTION_CONTROL
    _TRISA4 = 0; //Led
    _TRISB2 = 0; //Enable - Motor 1
    _TRISB3 = 0; //Enable - Motor 2
    _TRISB5 = 1;
    _TRISB6 = 1;
    _TRISB10 = 1;
    _TRISB11 = 1;
#else
#error Configuration error. Does not selected a board!
#endif
}

void InitLEDs(void) {
    led_controller[0].pin.pin = &led_1;
#if defined(UNAV_V1) || defined(ROBOCONTROLLER_V3)
    led_controller[1].pin.pin = &led_2;
#endif
#if defined(UNAV_V1)
    led_controller[2].pin.pin = &led_3;
    led_controller[3].pin.pin = &led_4;
#endif
    LED_Init(&FRQ_CPU, &led_controller[0], LED_NUM);
}

inline void UpdateBlink(short num, short blink) {
    LED_updateBlink(led_controller, num, blink);
}

void InitEvents(void) {
    init_events();
    
    EVENT_PRIORITY_LOW_ENABLE = 0;
    EVENT_PRIORITY_LOW_P = EVENT_PRIORITY_LOW_LEVEL;
    register_interrupt(EVENT_PRIORITY_LOW, &RTCIF);
    EVENT_PRIORITY_LOW_ENABLE = 1;
    
    EVENT_PRIORITY_MEDIUM_ENABLE = 0;
    EVENT_PRIORITY_MEDIUM_P = EVENT_PRIORITY_MEDIUM_LEVEL;
    register_interrupt(EVENT_PRIORITY_MEDIUM, &OC1IF);
    EVENT_PRIORITY_MEDIUM_ENABLE = 1;
    
    EVENT_PRIORITY_HIGH_ENABLE = 0;
    EVENT_PRIORITY_HIGH_P = EVENT_PRIORITY_HIGH_LEVEL;
    register_interrupt(EVENT_PRIORITY_HIGH, &OC2IF);
    EVENT_PRIORITY_HIGH_ENABLE = 1;
}

void __attribute__((interrupt, auto_psv)) _RTCCInterrupt(void) {
    event_manager(EVENT_PRIORITY_LOW);
    EVENT_PRIORITY_LOW_FLAG = 0; //interrupt flag reset
}

void __attribute__((interrupt, auto_psv)) _OC1Interrupt(void) {
    event_manager(EVENT_PRIORITY_MEDIUM);
    EVENT_PRIORITY_MEDIUM_FLAG = 0; // interrupt flag reset
}

void __attribute__((interrupt, auto_psv)) _OC2Interrupt(void) {
    event_manager(EVENT_PRIORITY_HIGH);
    EVENT_PRIORITY_HIGH_FLAG = 0; //interrupt flag reset
}

void __attribute__((interrupt, auto_psv)) _OC3Interrupt(void) {
    //event_manager(eventPriority priority);
    IFS1bits.OC3IF = 0;
}

void InitTimer1(void) {
    //T1CON = 10100000 00000000
    T1CONbits.TON = 0; // Disable Timer
    T1CONbits.TSIDL = 1; // Stop in Idle Mode bit
    T1CONbits.TGATE = 0; // Disable Gated Timer mode
    T1CONbits.TCKPS = 0b00; // Select 1:1 Prescaler
    T1CONbits.TSYNC = 0; // Disable Synchronization
    T1CONbits.TCS = 0; // Select internal clock source
    TMR1 = 0x00; // Clear timer register
    PR1 = TMR1_VALUE; // Load the period value

    IPC0bits.T1IP = SYS_TIMER_LEVEL; // Set Timer 1 Interrupt Priority Level
    IFS0bits.T1IF = 0; // Clear Timer 1 Interrupt Flag
    IEC0bits.T1IE = 1; // Enable Timer1 interrupt

    T1CONbits.TON = 1; // Start Timer
}

void __attribute__((interrupt, auto_psv)) _T1Interrupt(void) {
    /// Execution task manager
    task_manager();
    /// Blink controller for all LEDs
    LED_blinkController(&led_controller[0], LED_NUM);
    IFS0bits.T1IF = 0; // Clear Timer 1 Interrupt Flag
}






void set_process(uint8_t command, system_task_t process_state) {
    if (process_state.hashmap == HASHMAP_SYSTEM) {
        switch (command) {
            case SYSTEM_TASK_TIME:
                default_process[process_state.number].time = process_state.data;
                break;
            case SYSTEM_TASK_PRIORITY:
                default_process[process_state.number].priority = process_state.data;
                break;
            case SYSTEM_TASK_FRQ:
                default_process[process_state.number].frequency = process_state.data;
                break;
        }
    } else if (process_state.hashmap == HASHMAP_MOTOR) {
        switch (command) {
            case SYSTEM_TASK_TIME:
                motor_process[process_state.number].time = process_state.data;
                break;
            case SYSTEM_TASK_PRIORITY:
                motor_process[process_state.number].priority = process_state.data;
                break;
            case SYSTEM_TASK_FRQ:
                motor_process[process_state.number].frequency = process_state.data;
                break;
        }
    } else if (process_state.hashmap == HASHMAP_MOTION) {
        switch (command) {
            case SYSTEM_TASK_TIME:
                motion_process[process_state.number].time = process_state.data;
                break;
            case SYSTEM_TASK_PRIORITY:
                motion_process[process_state.number].priority = process_state.data;
                break;
            case SYSTEM_TASK_FRQ:
                motion_process[process_state.number].frequency = process_state.data;
                break;
        }
    }
}

system_task_t get_process(uint8_t command, system_task_t process_state) {
    if (process_state.hashmap == HASHMAP_SYSTEM) {
        switch (command) {
            case SYSTEM_TASK_TIME:
                process_state.data = default_process[process_state.number].time;
                break;
            case SYSTEM_TASK_PRIORITY:
                process_state.data = default_process[process_state.number].priority;
                break;
            case SYSTEM_TASK_FRQ:
                process_state.data = default_process[process_state.number].frequency;
                break;
            case SYSTEM_TASK_NUM:
                process_state.data = NUM_PROCESS_DEFAULT;
                break;
        }
    } else if (process_state.hashmap == HASHMAP_MOTOR) {
        switch (command) {
            case SYSTEM_TASK_TIME:
                process_state.data = motor_process[process_state.number].time;
                break;
            case SYSTEM_TASK_PRIORITY:
                process_state.data = motor_process[process_state.number].priority;
                break;
            case SYSTEM_TASK_FRQ:
                process_state.data = motor_process[process_state.number].frequency;
                break;
            case SYSTEM_TASK_NUM:
                process_state.data = PROCESS_MOTOR_LENGTH;
                break;
        }
    } else if (process_state.hashmap == HASHMAP_MOTION) {
        switch (command) {
            case SYSTEM_TASK_TIME:
                process_state.data = motion_process[process_state.number].time;
                break;
            case SYSTEM_TASK_PRIORITY:
                process_state.data = motion_process[process_state.number].priority;
                break;
            case SYSTEM_TASK_FRQ:
                process_state.data = motion_process[process_state.number].frequency;
                break;
            case SYSTEM_TASK_NUM:
                process_state.data = PROCESS_MOTION_LENGTH;
                break;
        }
    }
    return process_state;
}

system_task_name_t get_process_name(system_task_name_t process_name) {
    if (process_name.hashmap == HASHMAP_SYSTEM) {
        strcpy(process_name.data, default_process[process_name.number].name);
    } else if (process_name.hashmap == HASHMAP_MOTOR) {
        strcpy(process_name.data, motor_process[process_name.number].name);
    } else if (process_name.hashmap == HASHMAP_MOTION) {
        strcpy(process_name.data, motion_process[process_name.number].name);
    }
    return process_name;
}

//unsigned char update_priority(void) {
//    default_process[PROCESS_IDLE].time = 0;
//    InitInterrupts();
//    return PACKET_ACK;
//}

unsigned char update_frequency(void) {
    if (motor_process[LEFT_PROCESS_PID].frequency == 0 || motor_process[RIGHT_PROCESS_PID].frequency == 0) {
        EVENT_PRIORITY_MEDIUM_ENABLE = 0; // Disable Output Compare Channel 1 interrupt
    } else
        EVENT_PRIORITY_MEDIUM_ENABLE = 1; // Enable Output Compare Channel 1 interrupt
    if (motion_process[PROCESS_ODOMETRY].frequency == 0 || motion_process[PROCESS_VELOCITY].frequency == 0) {
        EVENT_PRIORITY_LOW_ENABLE = 0; // Disable RTC interrupt
    } else
        EVENT_PRIORITY_LOW_ENABLE = 1; // Enable RTC interrupt
    return PACKET_ACK;
}

system_service_t services(system_service_t service) {
    system_service_t service_send;
    service_send.command = service.command;
    switch (service.command) {
        case SERVICE_CODE_DATE:
            memcpy(service_send.buffer, version_date_, sizeof (version_date_));
            service_send.buffer[sizeof (version_date_) - 1] = ' ';
            memcpy(service_send.buffer + sizeof (version_date_), version_time_, sizeof (version_time_));
            break;
        case SERVICE_CODE_BOARD_NAME:
            memcpy(service_send.buffer, name_board, sizeof (name_board));
            break;
        case SERVICE_CODE_BOARD_TYPE:
            memcpy(service_send.buffer, type_board, sizeof (type_board));
            break;
        case SERVICE_CODE_VERSION:
            memcpy(service_send.buffer, version_code, sizeof (version_code));
            break;
        case SERVICE_CODE_AUTHOR:
            memcpy(service_send.buffer, author_code, sizeof (author_code));
            break;
        case SERVICE_RESET:
            if (reset_count < 3) {
                reset_count++;
            } else {
                SET_CPU_IPL(7); // disable all user interrupts
                //DelayN1ms(200);
                asm("RESET");
            }
            break;
        default:
            break;
    }
    return service_send;
}

void InitTimer2(void) {
    //T2CON = 10100000 00000000
    T2CONbits.TON = 0; // Disable Timer
    T2CONbits.TSIDL = 1; // Stop in Idle Mode bit
    T2CONbits.TGATE = 0; // Disable Gated Timer mode
    T2CONbits.TCKPS = 0b00; // Select 1:1 Prescaler
    T2CONbits.TCS = 0; // Select internal clock source
    TMR2 = 0x00; // Clear timer register
    PR2 = TMR2_VALUE; // Load the period value

    IPC1bits.T2IP = PWM_TIMER_LEVEL; // Set Timer 1 Interrupt Priority Level
    IFS0bits.T2IF = 0; // Clear Timer 1 Interrupt Flag
    IEC0bits.T2IE = 1; // Enable Timer1 interrupt

    T2CONbits.TON = 1; // Start Timer
}

void InitDMA0(void) {
    DMA0CNT = TOT_ADC_BUFF - 1; // 64 DMA request
    DMA0REQ = 13; // Select ADC1 as DMA Request source

    DMA0CONbits.AMODE = 2; // Peripheral Indirect Addressing mode
    DMA0CONbits.MODE = 0; // Continuous

    DMA0STA = __builtin_dmaoffset(AdcBuffer);
    DMA0PAD = (volatile unsigned int) &ADC1BUF0; // Point DMA to ADC1BUF0

    IFS0bits.DMA0IF = 0; // Clear DMA Interrupt Flag
    IPC1bits.DMA0IP = ADC_DMA_LEVEL; // Set DMA Interrupt Priority Level
    IEC0bits.DMA0IE = 1; // Enable DMA interrupt
    DMA0CONbits.CHEN = 1; // Enable DMA
}

void InitADC(void) {
    AD1CON1bits.FORM = 0; // Data Output Format: Integer
    AD1CON1bits.SSRC = 3; // Sample Clock Source: Internal counter sampling and starts convertions (auto-convert)
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
    AD1PCFGLbits.PCFG0 = 0; // AN0
    AD1PCFGLbits.PCFG1 = 0; // AN1

    IFS0bits.AD1IF = 0; // Clear the A/D interrupt flag bit
    IEC0bits.AD1IE = 0; // Do Not Enable A/D interrupt
    AD1CON1bits.ADON = 1; // module on
}
