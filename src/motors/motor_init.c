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

#include <xc.h>              /* Device header file */

#include <stdint.h>          /* For uint16_t definition                       */
#include <stdbool.h>         /* For true/false definition                     */
#include <pwm12.h>
#include <string.h>
#include <or_peripherals/GPIO/adc.h>

#include "system/peripherals.h"

#include "system/system.h"
#include "motors/motor_init.h"

#define DEFAULT_PWM_OFFSET 2048
#define DEFAULT_PWM_MAX 2047
#define DEFAULT_PWM_MIN -2048

#define GAIN_KILO 1000
#define ADC_AVSS 3.3
#define INTADC_MAX 1023
#define GAIN_ADC (ADC_AVSS/INTADC_MAX)
//#define GAIN_ADC (ADC_AVSS/INT16_MAX)
/**
 * Default value for motor parameters
 */
#define DEFAULT_CPR 300
#define DEFAULT_RATIO 30
#define DEFAULT_ENC_POSITION MOTOR_ENC_AFTER_GEAR
#define DEFAULT_ENC_CHANNELS MOTOR_ENC_CHANNEL_TWO
#define DEFAULT_ENC_Z_INDEX MOTOR_ENC_Z_INDEX_NO
#define DEFAULT_VERSUS_ROTATION MOTOR_ROTATION_COUNTERCLOCKWISE
#define DEFAULT_MOTOR_ENABLE MOTOR_ENABLE_LOW

/*****************************************************************************/
/* Global Variable Declaration                                               */
/*****************************************************************************/
typedef struct _icMode {
    short mode;
    short k;
} ICMode_t;

// Dynamic Interrupt Capture
const ICMode_t ICMode[4] = {
    {0b001, 1}, // 2X mode (default)
    {0b011, 2}, // 1X mode
    {0b100, 8}, // 1/4X mode
    {0b101, 32} // 1/16X mode
};
#define IC_DISABLE  0b000

#define ICMODE_DEFAULT 0
#define IC_TIMEPERIOD_TH_MAX 0x8000
#define IC_TIMEPERIOD_TH_MIN 2000

#ifdef UNAV_V1
const gpio_t enable[] = {
    GPIO_INIT(A, 7, GPIO_OUTPUT),   // ENABLE0
    GPIO_INIT(A, 10, GPIO_OUTPUT),   // ENABLE1
};
#elif ROBOCONTROLLER_V3
const gpio_t enable[] = {
    GPIO_INIT(A, 1, GPIO_OUTPUT),   // ENABLE0
    GPIO_INIT(A, 4, GPIO_OUTPUT),   // ENABLE1
};
#elif MOTION_CONTROL
const gpio_t enable[] = {
    GPIO_INIT(B, 2, GPIO_OUTPUT),   // ENABLE0
    GPIO_INIT(B, 3, GPIO_OUTPUT),   // ENABLE1
};
#endif

gpio_adc_t ADCmotor[2][2] = {
    // Current and Voltage Motor 0
    { GPIO_ADC(A, 0, AD1PCFGL, 0,          0, ADC_BUFF),
    GPIO_ADC(A, 1, AD1PCFGL, 1,     ADC_BUFF, ADC_BUFF)},
    // Current and Voltage Motor 1
    { GPIO_ADC(B, 0, AD1PCFGL, 2, 2*ADC_BUFF, ADC_BUFF),
    GPIO_ADC(B, 1, AD1PCFGL, 3,   3*ADC_BUFF, ADC_BUFF)},
};

/**
 * xc16 PID source in: folder_install_microchip_software/xc16/1.2x/src/libdsp.zip
 * on zip file: asm/pid.s
 */
fractional abcCoefficient[NUM_MOTORS][NUM_CONTROLLERS][3] __attribute__((section(".xbss, bss, xmemory")));
fractional controlHistory[NUM_MOTORS][NUM_CONTROLLERS][3] __attribute__((section(".ybss, bss, ymemory")));
typedef struct _motor_firmware {
    MOTOR_t motor;
    ICdata ICinfo;
} motor_firmware_t;

motor_firmware_t motor_fw[NUM_MOTORS];

/*****************************************************************************/
/* User Functions                                                            */
/*****************************************************************************/

motor_parameter_t Motor_init_parameters() {
    motor_parameter_t parameter;
    parameter.ratio = (float) DEFAULT_RATIO; //Gain to convert QEI value to rotation movement
    parameter.rotation = DEFAULT_VERSUS_ROTATION;
    parameter.bridge.enable = DEFAULT_MOTOR_ENABLE;
    parameter.bridge.pwm_dead_zone = 0;
    parameter.bridge.pwm_frequency = 0;
    parameter.bridge.volt_offset = 0;
    parameter.bridge.volt_gain = 6.06;
    parameter.bridge.current_offset = 0.8425;
    parameter.bridge.current_gain = 0.0623;
    parameter.encoder.cpr = DEFAULT_CPR; //Gain to convert input capture value to velocity
    parameter.encoder.type.position = DEFAULT_ENC_POSITION;
    parameter.encoder.type.channels = DEFAULT_ENC_CHANNELS;
    parameter.encoder.type.z_index = DEFAULT_ENC_Z_INDEX;
    return parameter;
}

void InitPWM(void) {
    // Holds the value to be loaded into dutycycle register
    unsigned int period;
    // Holds the value to be loaded into special event compare register
    unsigned int sptime;
    // Holds PWM configuration value
    unsigned int config1;
    // Holds the value be loaded into PWMCON1 register
    unsigned int config2;
    // Holds the value to configuration the special event trigger postscale and duty cycle
    unsigned int config3;
    // Config PWM
    period = 2048; // PWM F=19,340Hz counting UP 12bit resolution @ Fcy=39.628 MHz
    sptime = 0x0;
    // 1:1 postscaler, 1:1 prescale, free running mode
    // PWM time base ON, count up
    config1 = PWM1_DIS & PWM1_IDLE_CON & PWM1_OP_SCALE1 & PWM1_IPCLK_SCALE1 &
            PWM1_MOD_FREE;
    // PWM1H e PWM1L enabled in complementar mode
    // dsPICs with 3 pairs of PWM pins have one timer only (A)
    config2 = PWM1_MOD1_COMP & PWM1_PEN1L & PWM1_PEN1H &
            PWM1_MOD2_COMP & PWM1_PEN2L & PWM1_PEN2H &
            PWM1_PDIS3H & PWM1_PDIS3L;
    config3 = PWM1_SEVOPS1 & PWM1_OSYNC_PWM & PWM1_UEN;
    OpenMCPWM1(period, sptime, config1, config2, config3);
    // Dead Time Unit A assigned to both 1 & 2 PWM pairs
    /* SetMCPWM1DeadTimeAssignment(PWM1_DTS1A_UA & PWM1_DTS1I_UA & PWM1_DTS2A_UA & PWM1_DTS2I_UA); */
    P1DTCON2bits.DTS1A = 0;
    P1DTCON2bits.DTS1I = 0;
    P1DTCON2bits.DTS2A = 0;
    P1DTCON2bits.DTS2I = 0;
    // Dead time 100ns = 0.2% of PWM period
    SetMCPWM1DeadTimeGeneration(PWM1_DTA4 & PWM1_DTAPS1);
    // duty cycle reg=1, duty cycle=50% (motore fermo in LAP mode , update disable=0
    SetDCMCPWM1(1, 2048, 0);
    SetDCMCPWM1(2, 2048, 0);

    ConfigIntMCPWM1(PWM1_INT_DIS);
}

void InitQEI(MOTOR_t *motor) {
    switch (motor->index) {
        case MOTOR_ZERO:
            //QEI1CONbits.CNTERR= 0; // No position count error has occurred
            QEI1CONbits.QEISIDL = 1; // Discontinue module operation when device enters Idle mode
            QEI1CONbits.QEIM = 7; // Quadrature Encoder Interface enabled (x4 mode) with position counter reset by match (MAXxCNT)
            QEI1CONbits.SWPAB = (Motor_get_parameters(motor).rotation >= 1) ? 1 : 0; // Phase A and Phase B inputs swapped
            QEI1CONbits.PCDOUT = 0; // Position counter direction status output disabled (Normal I/O pin operation)
            //QEI1CONbits.TQGATE= 0  // Timer gated time accumulation disabled
            //QEI1CONbits.TQCKPS = 0b00	// 1:1 prescale value
            QEI1CONbits.POSRES = 0; // Index pulse does not reset position counter

            DFLT1CONbits.QEOUT = 1; // Digital filter outputs enabled on QEAx/QEBx/INDXx pins
            DFLT1CONbits.QECK = 6; // 1:128 Clock divide for QEAx/QEBx/INDXx

            MAX1CNT = 0xFFFF;
            POS1CNT = 0;
            break;
        case MOTOR_ONE:
            //QEI2CONbits.CNTERR= 0; // No position count error has occurred
            QEI2CONbits.QEISIDL = 1; // Discontinue module operation when device enters Idle mode
            QEI2CONbits.QEIM = 7; // Quadrature Encoder Interface enabled (x4 mode) with position counter reset by match (MAXxCNT)
            QEI2CONbits.SWPAB = (Motor_get_parameters(motor).rotation >= 1) ? 1 : 0; // Phase A and Phase B inputs swapped
            QEI2CONbits.PCDOUT = 0; // Position counter direction status output disabled (Normal I/O pin operation)
            //QEI2CONbits.TQGATE= 0  // Timer gated time accumulation disabled
            //QEI2CONbits.TQCKPS = 0b00	// 1:1 prescale value
            QEI2CONbits.POSRES = 0; // Index pulse does not reset position counter

            DFLT2CONbits.QEOUT = 1; // Digital filter outputs enabled on QEAx/QEBx/INDXx pins
            DFLT2CONbits.QECK = 6; // 1:128 Clock divide for QEAx/QEBx/INDXx

            MAX2CNT = 0xFFFF;
            POS2CNT = 0;
            break;
    }
}

void InitIC(MOTOR_t *motor) {
    switch (motor->index) {
        case MOTOR_ZERO:
            // Initialize Capture Module
            IC1CONbits.ICM = IC_DISABLE; // Disable Input Capture 1 module
            IC1CONbits.ICTMR = 1; // Select Timer2 as the IC1 Time base
            IC1CONbits.ICI = 0b00; // Interrupt on every second capture event
            IC1CONbits.ICM = ICMode[ICMODE_DEFAULT].mode; // Generate capture event on every Rising edge

            // Enable Capture Interrupt And Timer2
            IPC0bits.IC1IP = INPUT_CAPTURE_LEVEL; // Setup IC1 interrupt priority level
            IFS0bits.IC1IF = 0; // Clear IC1 Interrupt Status Flag
            IEC0bits.IC1IE = 1; // Enable IC1 interrupt
            break;
        case MOTOR_ONE:
            // Initialize Capture Module
            IC2CONbits.ICM = IC_DISABLE; // Disable Input Capture 2 module
            IC2CONbits.ICTMR = 1; // Select Timer2 as the IC1 Time base
            IC2CONbits.ICI = 0b00; // Interrupt on every second capture event
            IC2CONbits.ICM = ICMode[ICMODE_DEFAULT].mode; // Generate capture event on every Rising edge

            // Enable Capture Interrupt And Timer2
            IPC1bits.IC2IP = INPUT_CAPTURE_LEVEL; // Setup IC2 interrupt priority level
            IFS0bits.IC2IF = 0; // Clear IC2 Interrupt Status Flag
            IEC0bits.IC2IE = 1; // Enable IC2 interrupt
            break;
    }
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

void InitICinfo(ICdata *ICinfo) {
    //Input capture information
    ICinfo->k_mul = ICMode[ICMODE_DEFAULT].k;
    ICinfo->number = ICMODE_DEFAULT;
    ICinfo->SIG_VEL = 0;
    ICinfo->overTmr = 0;
    ICinfo->oldTime = 0;
    ICinfo->timePeriod = 0;
}

void Motor_Init(LED_controller_t* led_controller) {
    unsigned int i;
#ifdef UNAV_V1
    // Encoders
    _TRISB10 = 1;
    _TRISB11 = 1;
    _TRISB6 = 1;
    _TRISB5 = 1;
    _TRISB12 = 0; // PWM1 +
    _TRISB13 = 0; // PWM1 -
    _TRISB14 = 0; // PWM2 +
    _TRISB15 = 0; // PWM2 -
#elif ROBOCONTROLLER_V3
    // ADC
    _TRISB2 = 1; // CH1
    _TRISB3 = 1; // CH2
    _TRISC0 = 1; // CH3
    _TRISC1 = 1; // CH4
    // Encodes
    _TRISC6 = 1; // QEA_1
    _TRISC7 = 1; // QEB_1
    _TRISC8 = 1; // QEA_2
    _TRISC9 = 1; // QEB_2
#endif
    InitTimer2();               ///< Open Timer2 for InputCapture 1 & 2
    InitPWM();                  ///< Open PWM
    // Initialization motors
    for (i = 0; i < NUM_MOTORS; ++i) {
        // Initialization Input Capture
        InitICinfo(&motor_fw[i].ICinfo);
        // End
        InitQEI(&motor_fw[i].motor);                     ///< Open QEI
        InitIC(&motor_fw[i].motor);                      ///< Open Input Capture
        /// Initialize variables for motors
        Motor_init(&motor_fw[i].motor, i, 
            &abcCoefficient[i][0][0], &controlHistory[i][0][0], 
            &motor_fw[i].ICinfo, FRTMR2, &SelectIcPrescaler, 
            &SetDCMCPWM1, DEFAULT_PWM_OFFSET);
        // Register ADC
        Motor_register_adc(&motor_fw[i].motor, &ADCmotor[i][0], GAIN_ADC);
        // Register enable
        Motor_register_enable(&motor_fw[i].motor, &enable[i]);
        // Register led controller
        Motor_register_led_controller(&motor_fw[i].motor, led_controller);
        /// Initialize parameters for motors
        motor_parameter_t param = Motor_init_parameters();
        Motor_update_parameters(&motor_fw[i].motor, &param);
        // Initialize current PID controller
        motor_pid_t pid_current = {5, 0.001, 0.01, 1.0, 12000, false};
        Motor_update_pid(&motor_fw[i].motor, CONTROL_CURRENT, &pid_current);
        /// Initialize Velocity PID controller
        motor_pid_t pid_vel = { 6.0, 1.5, 0.2, 1.0, 1000, true};
        Motor_update_pid(&motor_fw[i].motor, CONTROL_VELOCITY, &pid_vel);
        /// Initialize Position PID controller
        motor_pid_t pid_pos = { 0.0, 0.0, 0.0, 0.0, 10, false};
        Motor_update_pid(&motor_fw[i].motor, CONTROL_POSITION, &pid_pos);
        /// Initialize safety procedure
        motor_safety_t safety = {400, 1000, 1000};
        Motor_update_safety(&motor_fw[i].motor, &safety);
        /// Initialize emergency procedure to stop
        motor_emergency_t emergency = {1.0, 2.0, 500};
        Motor_update_emergency(&motor_fw[i].motor, &emergency);
        /// Initialize constraints motor
        motor_t constraints = {MOTOR_CONTROL_MAX, MOTOR_CONTROL_MAX,
                    MOTOR_CONTROL_MAX, MOTOR_CONTROL_MAX, 0, 0};
        Motor_update_constraints(&motor_fw[i].motor, &constraints);
        /// Initialize state controller
        Motor_set_state(&motor_fw[i].motor, STATE_CONTROL_DISABLE);
        /// Run task controller
        Motor_run(&motor_fw[i].motor, RUN);
    }
}
/** 
 * Safely switch to the new Input Capture prescaler
 * @param motIdx number motor
 * @param mode
 */
inline void SwitchIcPrescaler(int motIdx, int mode) {
    // here is the assignment of the ICx module to the correct motor
    switch (motIdx) {
        case MOTOR_ZERO:
            IC1CONbits.ICM = IC_DISABLE;    // turn off pre scaler
            IC1CONbits.ICM = ICMode[mode].mode;  // Set new value for the Input Capture
            break;
        case MOTOR_ONE:
            IC2CONbits.ICM = IC_DISABLE;         // turn off pre scaler
            IC2CONbits.ICM = ICMode[mode].mode;  // Set new value for the Input Capture
            break;
    }
}

inline void SelectIcPrescaler(void *_motor) {
    MOTOR_t *motor = (MOTOR_t*) _motor;
    /** 
     * V = Kvel / timePeriod
     * is equal to:
     * timePeriod = Kvel / V = # Adimensional value
     * 
     * V -> inf , timePeriod -> 0   , ICmode -> 3 decrease pulses
     * V -> 0   , timePeriod -> inf , ICmode -> 0 increase pulses
     * 
     */
    int temp_number = 0;
    unsigned long doubletimePeriod = motor->ICinfo->delta;
    unsigned long halfPeriod = motor->ICinfo->delta;
    do {
        doubletimePeriod = doubletimePeriod * ICMode[temp_number].k;
        halfPeriod = halfPeriod / ICMode[temp_number].k;
        if (doubletimePeriod > IC_TIMEPERIOD_TH_MIN) {
            if (halfPeriod < IC_TIMEPERIOD_TH_MAX) {
                motor->ICinfo->k_mul = ICMode[temp_number].k;
                return;
            }
        }
        motor->ICinfo->k_mul = ICMode[temp_number].k;
        temp_number++;
    }while(temp_number <= 3);
}

void __attribute__((interrupt, no_auto_psv)) _IC1Interrupt(void) {
    // Run the Input Capture controller
    Motor_IC_controller(&motor_fw[MOTOR_ZERO].motor, &IC1BUF, QEI1CONbits.UPDN);
    // Clear the interrupt
    IFS0bits.IC1IF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _IC2Interrupt(void) {
    // Run the Input Capture controller
    Motor_IC_controller(&motor_fw[MOTOR_ONE].motor, &IC2BUF, QEI2CONbits.UPDN);
    // Clear the interrupt
    IFS0bits.IC2IF = 0;
}

void __attribute__((interrupt, auto_psv, shadow)) _T2Interrupt(void) {
    IFS0bits.T2IF = 0; // interrupt flag reset
    Motor_IC_timer(&motor_fw[MOTOR_ZERO].motor);
    Motor_IC_timer(&motor_fw[MOTOR_ONE].motor);
}