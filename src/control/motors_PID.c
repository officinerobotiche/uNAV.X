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

k_odo_t k_odo;
float wheel_m;

/**/
// From interrupt
extern volatile unsigned long timePeriodL; //Periodo Ruota Sinistra
extern volatile unsigned long timePeriodR; //Periodo Ruota Destra
extern volatile unsigned SIG_VELL; //Verso rotazione ruota Sinistra
extern volatile unsigned SIG_VELR; //Verso rotazione ruota Destra

//From high_level_control
extern volatile unsigned int control_state;

/******************************************************************************/
/* User Functions                                                             */

/******************************************************************************/

void init_parameter_motors(void) {
    int i;
    //Left motor parameters
    parameter_motor_left.k_vel = K_VEL; //Gain to convert input capture value to velocity
    parameter_motor_left.k_ang = K_ANG; //Gain to convert QEI value to rotation movement
    parameter_motor_left.versus = 1;
    parameter_motor_left.enable_set = false;
    //Right motor parameters
    parameter_motor_right.k_vel = K_VEL;
    parameter_motor_right.k_ang = K_ANG;
    parameter_motor_right.versus = 1;
    parameter_motor_right.enable_set = false;

    motor_left.control_vel = 0;
    motor_left.measure_vel = 0;
    motor_left.refer_vel = 0;
    motor_left.current = 0;
    motor_right.control_vel = 0;
    motor_right.measure_vel = 0;
    motor_right.refer_vel = 0;
    motor_right.current = 0;

    constraint.max_left = 25000;
    constraint.max_right = 25000;

    for (i = 0; i < NUM_MOTORS; ++i) {
        motor_state[i] = STATE_CONTROL_DISABLE;
        motor_ref[i] = 0;
        UpdateStateController(i, motor_state[i]);
    }

    update_parameter_motors();
}

void update_parameter_motors(void) {
    //Update encoder swap
    QEI1CONbits.SWPAB = (parameter_motor_left.versus >= 1) ? 1 : 0; // Phase A and Phase B inputs swapped
    QEI2CONbits.SWPAB = (parameter_motor_right.versus >= 1) ? 1 : 0; // Phase A and Phase B inputs swapped
}

void init_pid_control(void) {
    pid_left.kp = DEFAULT_KP;
    pid_left.ki = DEFAULT_KI;
    pid_left.kd = DEFAULT_KD;
    pid_right.kp = DEFAULT_KP;
    pid_right.ki = DEFAULT_KI;
    pid_right.kd = DEFAULT_KD;
}

void update_pid_l(void) {
    kCoeffs1[0] = Q15(pid_left.kp);
    kCoeffs1[1] = Q15(pid_left.ki);
    kCoeffs1[2] = Q15(pid_left.kd);
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

    switch (number) {
        case 0:
            if (abs(motor_ref[number]) > constraint.max_left) {
                motor_left.refer_vel = SGN((int) motor_ref[number]) * constraint.max_left;
            } else {
                motor_left.refer_vel = (int) motor_ref[number];
            }
            break;
        case 1:
            if (abs(motor_ref[number]) > constraint.max_right) {
                motor_right.refer_vel = SGN((int) motor_ref[number]) * constraint.max_right;
            } else {
                motor_right.refer_vel = (int) motor_ref[number];
            }
            break;
    }
    return TMR1 - t; // Time of esecution
}

void UpdateStateController(short num, motor_control_t state) {
    volatile bool enable = (state != STATE_CONTROL_DISABLE) ? true : false;
    /**
     * Set enable or disable motors
     */
    switch (num) {
        case -1:
            motor_state[0] = state;
            motor_state[1] = state;
            MOTOR_ENABLE1 = enable ^ parameter_motor_left.enable_set;
            MOTOR_ENABLE2 = enable ^ parameter_motor_right.enable_set;
            break;
        case 0:
            motor_state[num] = state;
            MOTOR_ENABLE1 = enable ^ parameter_motor_left.enable_set;
            break;
        case 1:
            motor_state[num] = state;
            MOTOR_ENABLE2 = enable ^ parameter_motor_right.enable_set;
            break;
    }
}

int MotorTaskController(void) {
    unsigned int t = TMR1; // Timing function
    short i;
    /**
     * If high level control selected, then set new reference for all motors.
     */
    if (control_state != STATE_CONTROL_HIGH_DISABLE)
        HighLevelTaskController();

    for (i = 0; i < NUM_MOTORS; ++i) {
        switch (motor_state[i]) {
            case STATE_CONTROL_DIRECT:
                //TODO To be implemented (Read issue #14)
                break;
            case STATE_CONTROL_POSITION:
                //TODO to be implemented
                break;
            case STATE_CONTROL_VELOCITY:
                /**
                 * Enable motors and check velocity constraint and save references
                 */
                MotorVelocityReference(i);
                break;
            case STATE_CONTROL_TORQUE:
                //TODO to be implemented
                break;
            default:
                break;
        }
    }

    return TMR1 - t; // Time of esecution
}

void SelectIcPrescaler(int motIdx) {

    if (motIdx == 0) {
        int16_t vel0 = motor_left.measure_vel;

        switch (IC1CONbits.ICM) {
            case IC_MODE0:
                if (vel0 >= MAX1) {
                    SwitchIcPrescaler(IC_MODE1, motIdx);
                }
                break;

            case IC_MODE1:
                if (vel0 < MIN1)
                {
                    SwitchIcPrescaler(IC_MODE0, motIdx);
                } 
                else if (vel0 >= MAX2)
                {
                    SwitchIcPrescaler(IC_MODE2, motIdx);
                }
                break;

            case IC_MODE2:
                if (vel0 < MIN2)
                {
                    SwitchIcPrescaler(IC_MODE1, motIdx);
                } 
                else if (vel0 >= MAX3)
                {
                    SwitchIcPrescaler(IC_MODE3, motIdx);
                }
                break;

            case IC_MODE3:
                if (vel0 < MIN3)
                {
                    SwitchIcPrescaler(IC_MODE2, motIdx);
                }
                break;

            default:
                SwitchIcPrescaler(IC_MODE0, motIdx);
                break;
        }
    }
    else
    {
        int16_t vel1 = motor_right.measure_vel;
        switch (IC2CONbits.ICM)
        {
            case IC_MODE0:
                if (vel1 >= MAX1)
                {
                    SwitchIcPrescaler(IC_MODE1, motIdx);
                }
                break;

            case IC_MODE1:
                if (vel1 < MIN1)
                {
                    SwitchIcPrescaler(IC_MODE0, motIdx);
                } 
                else if (vel1 >= MAX2)
                {
                    SwitchIcPrescaler(IC_MODE2, motIdx);
                }
                break;

            case IC_MODE2:
                if (vel1 < MIN2)
                {
                    SwitchIcPrescaler(IC_MODE1, motIdx);
                } 
                else if (vel1 >= MAX3)
                {
                    SwitchIcPrescaler(IC_MODE3, motIdx);
                }
                break;

            case IC_MODE3:
                if (vel1 < MIN3)
                {
                    SwitchIcPrescaler(IC_MODE2, motIdx);
                }
                break;

            default:
                SwitchIcPrescaler(IC_MODE0, motIdx);
                break;
        }
    }
}

int MotorPIDL(void) {
    unsigned int t = TMR1; // Timing

    unsigned long timePeriodLtmp;
    int SIG_VELLtmp;

    timePeriodLtmp = timePeriodL;
    timePeriodL = 0;
    SIG_VELLtmp = SIG_VELL;
    SIG_VELL = 0;
    motor_left.measure_vel = 0;

    PulsEncL += (int) POS1CNT; // Odometry
    POS1CNT = 0;

    // Speed calculation 
    if (SIG_VELLtmp)
        motor_left.measure_vel = SIG_VELLtmp * (parameter_motor_left.k_vel / timePeriodLtmp);
    PIDstruct1.controlReference = Q15(((float) motor_left.refer_vel) / constraint.max_left); // Setpoint
    PIDstruct1.measuredOutput = Q15(((float) motor_left.measure_vel) / constraint.max_left); // Measure

    PID(&PIDstruct1); // PID execution

    int pid_control = parameter_motor_left.versus * (PIDstruct1.controlOutput >> 4) + 2048; // PWM value
    // PWM output
    SetDCMCPWM1(1, pid_control, 0);

    // Control value calculation
    motor_left.control_vel = parameter_motor_left.versus * PIDstruct1.controlOutput;

    SelectIcPrescaler(0);

    return TMR1 - t; // Execution time
}

int MotorPIDR(void) {
    unsigned int t = TMR1; // Timing
    unsigned long timePeriodRtmp;
    int SIG_VELRtmp;

    timePeriodRtmp = timePeriodR;
    timePeriodR = 0;
    SIG_VELRtmp = SIG_VELR;
    SIG_VELR = 0;
    motor_right.measure_vel = 0;

    PulsEncR += (int) POS2CNT; // Odometry
    POS2CNT = 0;

    // Speed calculation
    if (SIG_VELRtmp) motor_right.measure_vel = SIG_VELRtmp * (parameter_motor_right.k_vel / timePeriodRtmp);
    PIDstruct2.controlReference = Q15(((float) motor_right.refer_vel) / constraint.max_right); // Setpoint
    PIDstruct2.measuredOutput = Q15(((float) motor_right.measure_vel) / constraint.max_right); // Measure

    PID(&PIDstruct2); // PID execution

    int pid_control = parameter_motor_right.versus * (PIDstruct2.controlOutput >> 4) + 2048; // PWM value
    // PWM output
    SetDCMCPWM1(2, pid_control, 0);

    // Control value calculation
    motor_right.control_vel = parameter_motor_right.versus * PIDstruct2.controlOutput;

    SelectIcPrescaler(1);

    return TMR1 - t; // Execution time
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
