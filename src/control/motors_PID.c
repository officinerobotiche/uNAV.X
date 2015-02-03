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

uint8_t k_mul; // k_vel multiplier according to IC scale

k_odo_t k_odo;
float wheel_m;

//variables for emergency
unsigned int counter_alive[NUM_MOTORS];
unsigned int counter_stop[NUM_MOTORS];
emergency_t emergency;
float emergency_step = 0;
float emergency_stop = 0;
bool save_velocity = true;
motor_t last_motor_left, last_motor_right;

/**/
// From interrupt
extern volatile unsigned long timePeriodL; // Left wheel period
extern volatile unsigned long timePeriodR; // Right wheel period
extern volatile unsigned SIG_VELL; // Left wheel rotation versus
extern volatile unsigned SIG_VELR; // Right wheel rotation versus

//From high_level_control
extern volatile unsigned int control_state;

//From System
extern parameter_system_t parameter_system;

/******************************************************************************/
/* User Functions                                                             */
/******************************************************************************/

parameter_motor_t init_parameter_motors(short num) {
    parameter_motor_t parameter;
    parameter.k_vel = K_VEL; //Gain to convert input capture value to velocity
    parameter.k_ang = K_ANG; //Gain to convert QEI value to rotation movement
    parameter.versus = 1;
    parameter.enable_set = false;
    switch (num) {
        case REF_MOTOR_LEFT:
            motor_left.control_vel = 0;
            motor_left.measure_vel = 0;
            motor_left.refer_vel = 0;
            motor_left.current = 0;

            constraint.max_left = 25000;
            break;
        case REF_MOTOR_RIGHT:
            motor_right.control_vel = 0;
            motor_right.measure_vel = 0;
            motor_right.refer_vel = 0;
            motor_right.current = 0;

            constraint.max_right = 25000;
            break;
    }
    k_mul = 1;
    motor_ref[num] = 0;
    return parameter;
}

void update_parameter_motors(short num, parameter_motor_t parameter) {
    //Update encoder swap
    switch (num) {
        case REF_MOTOR_LEFT:
            parameter_motor_left = parameter;
            QEI1CONbits.SWPAB = (parameter_motor_left.versus >= 1) ? 1 : 0; // Phase A and Phase B inputs swapped
            break;
        case REF_MOTOR_RIGHT:
            parameter_motor_right = parameter;
            QEI2CONbits.SWPAB = (parameter_motor_right.versus >= 1) ? 1 : 0; // Phase A and Phase B inputs swapped
            break;
    }
}

pid_control_t init_pid_control(short num) {
    pid_control_t pid;
    pid.kp = DEFAULT_KP;
    pid.ki = DEFAULT_KI;
    pid.kd = DEFAULT_KD;
    return pid;
}

void update_pid(short num, pid_control_t pid) {
    switch (num) {
        case REF_MOTOR_LEFT:
            pid_left = pid;
            kCoeffs1[0] = Q15(pid_left.kp);
            kCoeffs1[1] = Q15(pid_left.ki);
            kCoeffs1[2] = Q15(pid_left.kd);

            //Initialize the PID data structure: PIDstruct
            //Set up pointer to derived coefficients
            PIDstruct1.abcCoefficients = &abcCoefficient1[0];
            //Set up pointer to controller history samples
            PIDstruct1.controlHistory = &controlHistory1[0];
            // Clear the controler history and the controller output
            PIDInit(&PIDstruct1);
            //Derive the a,b, & c coefficients from the Kp, Ki & Kd
            PIDCoeffCalc(&kCoeffs1[0], &PIDstruct1);
            break;
        case REF_MOTOR_RIGHT:
            pid_right = pid;
            kCoeffs2[0] = Q15(pid_right.kp);
            kCoeffs2[1] = Q15(pid_right.ki);
            kCoeffs2[2] = Q15(pid_right.kd);

            //Initialize the PID data structure: PIDstruct
            //Set up pointer to derived coefficients
            PIDstruct2.abcCoefficients = &abcCoefficient2[0];
            //Set up pointer to controller history samples
            PIDstruct2.controlHistory = &controlHistory2[0];
            // Clear the controler history and the controller output
            PIDInit(&PIDstruct2);
            //Derive the a,b, & c coefficients from the Kp, Ki & Kd
            PIDCoeffCalc(&kCoeffs2[0], &PIDstruct2);
            break;
    }
}

emergency_t init_parameter_emergency(short num) {
    emergency_t emer;
    counter_alive[num] = 0;
    counter_stop[num] = 0;
    emer.time = 1.0;
    emer.timeout = 500;
    emer.stop = 2.0;
    return emer;
}

void update_parameter_emergency(short num, emergency_t emergency_data) {
    emergency = emergency_data;
    emergency_step = emergency.time / FRTMR1;
    emergency_stop = emergency.stop * FRTMR1;
}

int MotorVelocityReference(short number) {
    unsigned int t = TMR1; // Timing function

    switch (number) {
        case REF_MOTOR_LEFT:
            if (abs(motor_ref[number]) > constraint.max_left) {
                motor_left.refer_vel = SGN((int) motor_ref[number]) * constraint.max_left;
            } else {
                motor_left.refer_vel = (int) motor_ref[number];
            }
            break;
        case REF_MOTOR_RIGHT:
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
    int led_state = (state != STATE_CONTROL_EMERGENCY) ? state + 1 : state;
    /**
     * Set enable or disable motors
     */
    switch (num) {
        case -1:
            motor_state[0] = state;
            motor_state[1] = state;
            MOTOR_ENABLE1 = enable ^ parameter_motor_left.enable_set;
            MOTOR_ENABLE2 = enable ^ parameter_motor_right.enable_set;
#ifndef MOTION_CONTROL
            UpdateBlink(0, led_state);
            UpdateBlink(1, led_state);
#endif
            break;
        case REF_MOTOR_LEFT:
            motor_state[num] = state;
            MOTOR_ENABLE1 = enable ^ parameter_motor_left.enable_set;
#ifndef MOTION_CONTROL
            UpdateBlink(num, led_state);
#endif
            break;
        case REF_MOTOR_RIGHT:
            motor_state[num] = state;
            MOTOR_ENABLE2 = enable ^ parameter_motor_right.enable_set;
#ifndef MOTION_CONTROL
            UpdateBlink(num, led_state);
#endif
            break;
    }
#ifdef MOTION_CONTROL
    UpdateBlink(LED1, led_state);
#endif
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
            case STATE_CONTROL_EMERGENCY:
                /**
                 * Set motor in emergency mode
                 */
                Emergency(i);
                break;
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
        // Emergency controller
        if (motor_state[i] > STATE_CONTROL_DISABLE) {
            if ((counter_alive[i] + 1) >= emergency.timeout) {
                /**
                 * Set Motor in emergency mode
                 */
                UpdateStateController(i, STATE_CONTROL_EMERGENCY);
                /**
                 * Save state motor
                 */
                switch (i) {
                    case 0:
                        last_motor_left.refer_vel = motor_left.refer_vel;
                        break;
                    case 1:
                        last_motor_right.refer_vel = motor_right.refer_vel;
                        break;
                }
                counter_stop[i] = 0;
                counter_alive[i] = 0;
            } else
                counter_alive[i]++;
        }
    }

    return TMR1 - t; // Time of esecution
}

void SelectIcPrescaler(int motIdx) {

    if (motIdx == 0) {
        int16_t vel0 = abs(motor_left.measure_vel);

        switch (IC1CONbits.ICM) {
            case IC_MODE0:
                if (vel0 >= MAX1) {
                    k_mul = 2;
                    SwitchIcPrescaler(1, motIdx);
                }
                break;

            case IC_MODE1:
                if (vel0 < MIN1) {
                    k_mul = 1;
                    SwitchIcPrescaler(0, motIdx);
                } else if (vel0 >= MAX2) {
                    k_mul = 8;
                    SwitchIcPrescaler(2, motIdx);
                }
                break;

            case IC_MODE2:
                if (vel0 < MIN2) {
                    k_mul = 2;
                    SwitchIcPrescaler(1, motIdx);
                } else if (vel0 >= MAX3) {
                    k_mul = 32;
                    SwitchIcPrescaler(3, motIdx);
                }
                break;

            case IC_MODE3:
                if (vel0 < MIN3) {
                    k_mul = 8;
                    SwitchIcPrescaler(2, motIdx);
                }
                break;

            default:
                k_mul = 1;
                SwitchIcPrescaler(0, motIdx);
                break;
        }
    } else {
        int16_t vel1 = abs(motor_right.measure_vel);
        switch (IC2CONbits.ICM) {
            case IC_MODE0:
                if (vel1 >= MAX1) {
                    k_mul = 2;
                    SwitchIcPrescaler(1, motIdx);
                }
                break;

            case IC_MODE1:
                if (vel1 < MIN1) {
                    k_mul = 1;
                    SwitchIcPrescaler(0, motIdx);
                } else if (vel1 >= MAX2) {
                    k_mul = 8;
                    SwitchIcPrescaler(2, motIdx);
                }
                break;

            case IC_MODE2:
                if (vel1 < MIN2) {
                    k_mul = 2;
                    SwitchIcPrescaler(1, motIdx);
                } else if (vel1 >= MAX3) {
                    k_mul = 32;
                    SwitchIcPrescaler(3, motIdx);
                }
                break;

            case IC_MODE3:
                if (vel1 < MIN3) {
                    k_mul = 8;
                    SwitchIcPrescaler(2, motIdx);
                }
                break;

            default:
                k_mul = 1;
                SwitchIcPrescaler(0, motIdx);
                break;
        }
    }
}

#define INV_PID_TIME 1000 // TODO replace with real dT = 1/f_pid

void measureVelocity(short num) {
    unsigned long timePeriodtmp;
    int SIG_VELtmp;

    switch (num) {
        case REF_MOTOR_LEFT:
            timePeriodtmp = timePeriodL;
            timePeriodL = 0;
            SIG_VELtmp = SIG_VELL;
            SIG_VELL = 0;
            motor_left.measure_vel = 0;

            PulsEncL += (int) POS1CNT; // Odometry
            //int dAng = (int) POS1CNT * parameter_motor_left.k_vel * k_mul; // Odometry to angular
            POS1CNT = 0;
            // Speed calculation 
            if (SIG_VELtmp) {
                int16_t ic_contrib = SIG_VELtmp * ((parameter_motor_left.k_vel * k_mul) / timePeriodtmp);
                //int16_t odo_contrib = dAng* INV_PID_TIME; // TODO replace with real dT = 1/f_pid
                // motor_left.measure_vel = (ic_contrib + odo_contrib)/2;
                motor_left.measure_vel = ic_contrib;
                int a = 0;
                if (abs(motor_left.measure_vel) > 10000)
                    a++;
            }
            //SelectIcPrescaler(0,motor_left.measure_vel);
            break;
        case REF_MOTOR_RIGHT:
            timePeriodtmp = timePeriodR;
            timePeriodR = 0;
            SIG_VELtmp = SIG_VELR;
            SIG_VELR = 0;
            motor_right.measure_vel = 0;

            PulsEncR += (int) POS2CNT; // Odometry
            POS2CNT = 0;

            // Speed calculation
            if (SIG_VELtmp)
                motor_right.measure_vel = SIG_VELtmp * (parameter_motor_right.k_vel / (timePeriodtmp * k_mul));
            //SelectIcPrescaler(1,motor_right.measure_vel);
            break;
    }
}

int MotorPIDL(void) {
    unsigned int t = TMR1; // Timing

    PIDstruct1.controlReference = Q15(((float) motor_left.refer_vel) / constraint.max_left); // Setpoint
    PIDstruct1.measuredOutput = Q15(((float) motor_left.measure_vel) / constraint.max_left); // Measure

    PID(&PIDstruct1); // PID execution
    // Control value calculation
    motor_left.control_vel = parameter_motor_left.versus * PIDstruct1.controlOutput;

    int pid_control = (motor_left.control_vel >> 4) + 2048; // PWM value
    // PWM output
    SetDCMCPWM1(1, pid_control, 0);

    return TMR1 - t; // Execution time
}

int MotorPIDR(void) {
    unsigned int t = TMR1; // Timing

    PIDstruct2.controlReference = Q15(((float) motor_right.refer_vel) / constraint.max_right); // Setpoint
    PIDstruct2.measuredOutput = Q15(((float) motor_right.measure_vel) / constraint.max_right); // Measure

    PID(&PIDstruct2); // PID execution
    // Control value calculation
    motor_right.control_vel = parameter_motor_right.versus * PIDstruct2.controlOutput;

    int pid_control = (motor_right.control_vel >> 4) + 2048; // PWM value
    // PWM output
    SetDCMCPWM1(2, pid_control, 0);

    return TMR1 - t; // Execution time
}

bool Emergency(short num) {
    switch (num) {
        case REF_MOTOR_LEFT:
            motor_left.refer_vel -= emergency_step * last_motor_left.refer_vel;
            //motor_ref[num] = motor_left.refer_vel;
            if (SGN(last_motor_left.refer_vel) * motor_left.refer_vel < 0)
                motor_left.refer_vel = 0;
            if (motor_left.refer_vel == 0) {
                if ((counter_stop[num] + 1) >= emergency_stop) {
                    UpdateStateController(num, STATE_CONTROL_DISABLE);
                    counter_stop[num] = 0;
                } else
                    counter_stop[num]++;
            }
            break;
        case REF_MOTOR_RIGHT:
            motor_right.refer_vel -= emergency_step * last_motor_right.refer_vel;
            //motor_ref[num] = motor_right.refer_vel;
            if (SGN(last_motor_right.refer_vel) * motor_right.refer_vel < 0)
                motor_right.refer_vel = 0;
            if (motor_right.refer_vel == 0) {
                if ((counter_stop[num] + 1) >= emergency_stop) {
                    UpdateStateController(num, STATE_CONTROL_DISABLE);
                    counter_stop[num] = 0;
                } else
                    counter_stop[num]++;
            }
            break;
    }
    return true;
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
