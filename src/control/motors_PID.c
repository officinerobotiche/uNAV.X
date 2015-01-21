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
#include <dsp.h>
#include <pwm12.h>
#include "control/high_level_control.h"
#include "control/motors_PID.h"       /* variables/params used by motorsPID.c         */
#include "system/system.h"
#include "system/user.h"
#include "packet/packet.h"

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/
/**
 * xc16 PID source in: folder_install_microchip_software/xc16/1.2x/src/libdsp.zip
 * on zip file: asm/pid.s
 */
tPID PIDstruct1; // PID motore Sinistra
fractional abcCoefficient1[3] __attribute__((section(".xbss, bss, xmemory")));
fractional controlHistory1[3] __attribute__((section(".ybss, bss, ymemory")));
fractional kCoeffs1[3]; //Coefficienti KP, KI, KD Per PID1 Sinistra
tPID PIDstruct2; //PID motore Destra
fractional abcCoefficient2[3] __attribute__((section(".xbss, bss, xmemory")));
fractional controlHistory2[3] __attribute__((section(".ybss, bss, ymemory")));
fractional kCoeffs2[3]; //Coefficienti KP, KI, KD Per PID2 Destra

// ADC buffer, 2 channels (AN0, AN1), 32 bytes each, 2 x 32 = 64 bytes
int AdcBuffer[2][ADC_BUFF] __attribute__((space(dma), aligned(256)));

/** */

volatile int PulsEncL = 0; //Buffer for deadReckoning
volatile int PulsEncR = 0; //Buffer for deadReckoning

parameter_motor_t parameter_motor_left, parameter_motor_right;
constraint_t constraint;
pid_control_t pid_left, pid_right;
motor_control_t motor_ref[NUM_MOTORS];
motor_control_t motor_state[NUM_MOTORS];
motor_t motor_left, motor_right;

typedef struct parameter_vel {
    float k_vel;
    int sign;
} parameter_vel_t;

k_odo_t k_odo;
float wheel_m;

parameter_vel_t motor_vel_left, motor_vel_right;

unsigned int control_motor_state[NUM_MOTORS];

/**/
// From interrupt
extern volatile unsigned long timePeriodL; //Periodo Ruota Sinistra
extern volatile unsigned long timePeriodR; //Periodo Ruota Destra
extern volatile unsigned SIG_VELL; //Verso rotazione ruota Sinistra
extern volatile unsigned SIG_VELR; //Verso rotazione ruota Destra

//From high_level_control
extern volatile unsigned int control_state;

//From interrupt
extern unsigned int counter_stop;

/******************************************************************************/
/* User Functions                                                             */

/******************************************************************************/

void init_parameter_motors(void) {
    int i;
    //Left motor parameters
    parameter_motor_left.k_vel = K_VEL; //Gain to convert input capture value to velocity
    parameter_motor_left.k_ang = K_ANG; //Gain to convert QEI value to rotation movement
    parameter_motor_left.versus = false;
    parameter_motor_left.enable_set = false;
    //Right motor parameters
    parameter_motor_right.k_vel = K_VEL;
    parameter_motor_right.k_ang = K_ANG;
    parameter_motor_right.versus = false;
    parameter_motor_right.enable_set = false;

    motor_left.control_vel = 0;
    motor_left.measure_vel = 0;
    motor_left.refer_vel = 0;
    motor_left.current = 0;
    motor_right.control_vel = 0;
    motor_right.measure_vel = 0;
    motor_right.refer_vel = 0;
    motor_right.current = 0;

    constraint.max_left = 14000;
    constraint.max_right = 14000;

    for (i = 0; i < NUM_MOTORS; ++i) {
        motor_state[i] = STATE_CONTROL_DISABLE;
        motor_ref[i] = 0;
        UpdateStateController(i, motor_state[i]);
    }
}

void update_parameter_motors(void) {
    motor_vel_left.k_vel = parameter_motor_left.k_vel;
    motor_vel_right.k_vel = parameter_motor_right.k_vel;
    //Update encoder swap
    motor_vel_left.sign = parameter_motor_left.versus;
    motor_vel_right.sign = parameter_motor_right.versus;
    QEI1CONbits.SWPAB = (parameter_motor_left.versus >= 1) ? 1 : 0; // Phase A and Phase B inputs swapped
    QEI2CONbits.SWPAB = (parameter_motor_right.versus >= 1) ? 1 : 0; // Phase A and Phase B inputs swapped
}

void init_pid_control(void) {
    pid_left.kp = 0.6;
    pid_left.ki = 0.7;
    pid_left.kd = 0.1;
    pid_right.kp = 0.6;
    pid_right.ki = 0.7;
    pid_right.kd = 0.1;
}

void update_pid_l(void) {
    kCoeffs1[0] = Q15(pid_left.kp); //0.5
    kCoeffs1[1] = Q15(pid_left.ki); //0.6
    kCoeffs1[2] = Q15(pid_left.kd); //0.0
    InitPid1(); //Init PIDL
}

void update_pid_r(void) {
    kCoeffs2[0] = Q15(pid_right.kp);
    kCoeffs2[1] = Q15(pid_right.ki);
    kCoeffs2[2] = Q15(pid_right.kd);
    InitPid2(); //Init PIDR
}

void InitPid1(void) {
    //Initialize the PID data structure: PIDstruct
    //Set up pointer to derived coefficients
    PIDstruct1.abcCoefficients = &abcCoefficient1[0];
    //Set up pointer to controller history samples
    PIDstruct1.controlHistory = &controlHistory1[0];
    // Clear the controler history and the controller output
    PIDInit(&PIDstruct1);
    //Derive the a,b, & c coefficients from the Kp, Ki & Kd
    PIDCoeffCalc(&kCoeffs1[0], &PIDstruct1);
}

void InitPid2(void) {
    //Initialize the PID data structure: PIDstruct
    //Set up pointer to derived coefficients
    PIDstruct2.abcCoefficients = &abcCoefficient2[0];
    //Set up pointer to controller history samples
    PIDstruct2.controlHistory = &controlHistory2[0];
    // Clear the controler history and the controller output
    PIDInit(&PIDstruct2);
    //Derive the a,b, & c coefficients from the Kp, Ki & Kd
    PIDCoeffCalc(&kCoeffs2[0], &PIDstruct2);
}

int MotorVelocityReference(short number) {
    unsigned int t = TMR1; // Timing function

    if (abs(motor_ref[number]) > constraint.max_left) {
        motor_left.refer_vel = SGN((int) motor_ref[number]) * constraint.max_left;
    } else {
        motor_left.refer_vel = (int) motor_ref[number];
    }
    return TMR1 - t; // Time of esecution
}

void UpdateStateController(short num, motor_control_t motor) {
    /**
     * Set enable or disable motors
     */
    switch (num) {
        case -1:
            motor_state[0] = (motor > 0) ? true : false;
            motor_state[1] = (motor > 0) ? true : false;
            MOTOR_ENABLE1 = motor_state[0] ^ parameter_motor_left.enable_set;
            MOTOR_ENABLE2 = motor_state[1] ^ parameter_motor_right.enable_set;
            break;
        default:
            motor_state[num] = (motor > 0) ? true : false;
            MOTOR_ENABLE1 = motor_state[num] ^ parameter_motor_left.enable_set;
            break;
    }
    /**
     * Reset time emergency
     */
    counter_stop = 0;
}

int MotorTaskController(void) {
    unsigned int t = TMR1; // Timing function
    short i;
    /**
     * If high level control selected, then set new reference for all motors.
     */
    if (control_state > STATE_CONTROL_HIGH_DISABLE)
        HighLevelTaskController();

    for (i = 0; i < NUM_MOTORS; ++i) {
        switch (control_motor_state[i]) {
            case STATE_CONTROL_DIRECT:
                //TODO To be implement (Read issue #14)
                break;
            case STATE_CONTROL_POSITION:
                //TODO to be implement
                break;
            case STATE_CONTROL_VELOCITY:
                /**
                 * Enable motors and check velocity constraint and save references
                 */
                MotorVelocityReference(i);
                break;
            case STATE_CONTROL_TORQUE:
                //TODO to be implement
                break;
            default:
                break;
        }
    }

    return TMR1 - t; // Time of esecution
}

int MotorPIDL(void) {
    unsigned int t = TMR1; //Timing funzione
    unsigned long timePeriodLtmp; //Variabili temporanee Periodo
    int SIG_VELLtmp; //Segno velocità

    timePeriodLtmp = timePeriodL; //Salvataggio TimerPeriod
    timePeriodL = 0; //Pulizia variabile
    SIG_VELLtmp = SIG_VELL; //Salvataggio Segno velocità
    SIG_VELL = 0; //Pulizia variabile
    motor_left.measure_vel = 0; //Flush variaibile velocità R

    PulsEncL += (int) POS1CNT; //Salvataggio spazio percorso
    POS1CNT = 0; //Reset registro
    //calcolo della velocità
    //Verifica SIG_VELLtmp!=0 & calcolo velocità
    if (SIG_VELLtmp) motor_left.measure_vel = SIG_VELLtmp * (parameter_motor_left.k_vel / timePeriodLtmp);
    PIDstruct1.controlReference = motor_left.refer_vel; //Riferimento Ruota Sinistra
    PIDstruct1.measuredOutput = motor_left.measure_vel; //Misura velocità
    PID(&PIDstruct1); //Esecuzione funzione PID
    int pid_control = motor_vel_left.sign * (PIDstruct1.controlOutput >> 4) + 2049; //Conversione valore per PWM
    //Invio dell'azione di controllo al motore per mezzo del PWM
    SetDCMCPWM1(1, pid_control, 0);
    motor_left.control_vel = motor_vel_left.sign * PIDstruct1.controlOutput;

    return TMR1 - t; //Misura tempo di esecuzione
}

int MotorPIDR(void) {
    unsigned int t = TMR1; //Timing funzione
    unsigned long timePeriodRtmp; //Variabili temporanee Periodo
    int SIG_VELRtmp; //Segno velocità

    timePeriodRtmp = timePeriodR; //Salvataggio TimerPeriod
    timePeriodR = 0; //Pulizia variabile
    SIG_VELRtmp = SIG_VELR; //Salvataggio Segno velocità
    SIG_VELR = 0; //Pulizia variabile
    motor_right.measure_vel = 0; //Flush variabile velocità R

    PulsEncR += (int) POS2CNT; //Salvataggio spazio percorso
    POS2CNT = 0; //Reset registro
    //calcolo della velocità
    //Verifica SIG_VELLtmp!=0 & calcolo velocità
    if (SIG_VELRtmp) motor_right.measure_vel = SIG_VELRtmp * (parameter_motor_right.k_vel / timePeriodRtmp);
    PIDstruct2.controlReference = motor_right.refer_vel; //Riferimento Ruota Destra
    PIDstruct2.measuredOutput = motor_right.measure_vel; //Misura velocità
    PID(&PIDstruct2); //Esecuzione funzione PID
    int pid_control = motor_vel_right.sign * (PIDstruct2.controlOutput >> 4) + 2049; //Conversione valore per PWM
    //Invio dell'azione di controllo al motore per mezzo del PWM
    SetDCMCPWM1(2, pid_control, 0);
    motor_right.control_vel = motor_vel_right.sign * PIDstruct2.controlOutput;

    return TMR1 - t; //Misura tempo di esecuzione
}

void adc_motors_current(void) {
    int AdcCount = 0; //Counter
    long ADCValueTmp[ADC_CHANNELS] = {0, 0}; //Array to filter ADC data

    for (AdcCount = 0; AdcCount < ADC_BUFF; AdcCount++) // Evaluate mean value
    {
        ADCValueTmp[0] += AdcBuffer[0][AdcCount]; //Sum for AN0
        ADCValueTmp[1] += AdcBuffer[1][AdcCount]; //Sum for AN1
    }
    motor_left.current = ADCValueTmp[0] >> 6; //Shift
    motor_right.current = ADCValueTmp[1] >> 6; //Shift
}
