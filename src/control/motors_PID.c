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
//tPID PIDstruct1; // PID motore Sinistra
fractional abcCoefficient1[3] __attribute__((section(".xbss, bss, xmemory")));
fractional controlHistory1[3] __attribute__((section(".ybss, bss, ymemory")));
fractional kCoeffs1[3]; //Coefficienti KP, KI, KD Per PID1 Sinistra
//tPID PIDstruct2; //PID motore Destra
fractional abcCoefficient2[3] __attribute__((section(".xbss, bss, xmemory")));
fractional controlHistory2[3] __attribute__((section(".ybss, bss, ymemory")));
fractional kCoeffs2[3]; //Coefficienti KP, KI, KD Per PID2 Destra

// ADC buffer, 2 channels (AN0, AN1), 32 bytes each, 2 x 32 = 64 bytes
int AdcBuffer[2][ADC_BUFF] __attribute__((space(dma), aligned(256)));

/** */

typedef struct new_motor {
    //Use ONLY in firmware
    uint8_t k_mul; // k_vel multiplier according to IC scale
    volatile int PulsEnc; //Buffer for deadReckoning
    //Common
    state_controller_t state;
    parameter_motor_t parameter_motor;
    pid_control_t pid;
    tPID PIDstruct;
} new_motor_t;
new_motor_t motors[NUM_MOTORS];

#define NUM_VALUES 10
int16_t velocity[NUM_VALUES];

constraint_t constraint;
//pid_control_t pid_left, pid_right;
//motor_control_t motor_ref[NUM_MOTORS];
//state_controller_t motor_state[NUM_MOTORS];
motor_t motor_left, motor_right;



k_odo_t k_odo;
float wheel_m;

//variables for emergency
unsigned int counter_alive[NUM_MOTORS];
unsigned int counter_stop[NUM_MOTORS];
emergency_t emergency;
float emergency_step = 0;
float emergency_stop = 0;
bool save_velocity = true;
float last_motor_left, last_motor_right;

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
    motors[num].k_mul = 1;
    return parameter;
}

void update_parameter_motors(short num, parameter_motor_t parameter) {
    //Update parameter configuration
    motors[num].parameter_motor = parameter;
    //Update encoder swap
    switch (num) {
        case REF_MOTOR_LEFT:
            QEI1CONbits.SWPAB = (motors[num].parameter_motor.versus >= 1) ? 1 : 0; // Phase A and Phase B inputs swapped
            break;
        case REF_MOTOR_RIGHT:
            QEI2CONbits.SWPAB = (motors[num].parameter_motor.versus >= 1) ? 1 : 0; // Phase A and Phase B inputs swapped
            break;
    }
}

/* inline */ parameter_motor_t get_parameter_motor(short motIdx) {
    return motors[motIdx].parameter_motor;
}

pid_control_t init_pid_control(short num) {
    pid_control_t pid;
    pid.kp = DEFAULT_KP;
    pid.ki = DEFAULT_KI;
    pid.kd = DEFAULT_KD;
    return pid;
}

void update_pid(short num, pid_control_t pid) {
    // Update value of pid
    motors[num].pid = pid;
    switch (num) {
        case REF_MOTOR_LEFT:
            kCoeffs1[0] = Q15(motors[num].pid.kp);
            kCoeffs1[1] = Q15(motors[num].pid.ki);
            kCoeffs1[2] = Q15(motors[num].pid.kd);

            //Initialize the PID data structure: PIDstruct
            //Set up pointer to derived coefficients
            motors[num].PIDstruct.abcCoefficients = &abcCoefficient1[0];
            //Set up pointer to controller history samples
            motors[num].PIDstruct.controlHistory = &controlHistory1[0];
            // Clear the controler history and the controller output
            PIDInit(&motors[num].PIDstruct);
            //Derive the a,b, & c coefficients from the Kp, Ki & Kd
            PIDCoeffCalc(&kCoeffs1[0], &motors[num].PIDstruct);
            break;
        case REF_MOTOR_RIGHT:
            kCoeffs2[0] = Q15(motors[num].pid.kp);
            kCoeffs2[1] = Q15(motors[num].pid.ki);
            kCoeffs2[2] = Q15(motors[num].pid.kd);

            //Initialize the PID data structure: PIDstruct
            //Set up pointer to derived coefficients
            motors[num].PIDstruct.abcCoefficients = &abcCoefficient2[0];
            //Set up pointer to controller history samples
            motors[num].PIDstruct.controlHistory = &controlHistory2[0];
            // Clear the controler history and the controller output
            PIDInit(&motors[num].PIDstruct);
            //Derive the a,b, & c coefficients from the Kp, Ki & Kd
            PIDCoeffCalc(&kCoeffs2[0], &motors[num].PIDstruct);
            break;
    }
}

/* inline */ pid_control_t get_pid_value(short motIdx) {
    return motors[motIdx].pid;
}

emergency_t init_parameter_emergency(short num) {
    emergency_t emer;
    counter_alive[num] = 0;
    counter_stop[num] = 0;
    emergency.slope_time = 1.0;
    emergency.timeout = 500;
    emergency.bridge_off = 2.0;
    return emer;
}

void update_parameter_emergency(short num, emergency_t emergency_data) {
    emergency = emergency_data;
    emergency_step = emergency.slope_time * FRTMR1;
    emergency_stop = emergency.bridge_off * FRTMR1;
}

int MotorVelocityReference(short number) {
    unsigned int t = TMR1; // Timing function

    switch (number) {
        case REF_MOTOR_LEFT:
            if (abs(motor_left.refer_vel) > constraint.max_left) {
                motor_left.refer_vel = SGN(motor_left.refer_vel) * constraint.max_left;
            }
            break;
        case REF_MOTOR_RIGHT:
            if (abs(motor_right.refer_vel) > constraint.max_right) {
                motor_right.refer_vel = SGN(motor_right.refer_vel) * constraint.max_right;
            }
            break;
    }
    return TMR1 - t; // Time of esecution
}

/* inline */ int get_pulse_encoder(short motIdx) {
    return motors[motIdx].PulsEnc;
}

void UpdateStateController(short num, motor_control_t state) {
    volatile bool enable = (state != STATE_CONTROL_DISABLE) ? true : false;
    int led_state = (state != STATE_CONTROL_EMERGENCY) ? state + 1 : state;
    /**
     * Set enable or disable motors
     */

    switch (num) {
        case -1:
            motors[0].state = state;
            motors[1].state = state;
            MOTOR_ENABLE1 = enable ^ motors[0].parameter_motor.enable_set;
            MOTOR_ENABLE2 = enable ^ motors[1].parameter_motor.enable_set;
#ifndef MOTION_CONTROL
            UpdateBlink(0, led_state);
            UpdateBlink(1, led_state);
#endif
            break;
        case REF_MOTOR_LEFT:
            motors[num].state = state;
            MOTOR_ENABLE1 = enable ^ motors[num].parameter_motor.enable_set;
            if(state == STATE_CONTROL_EMERGENCY) {
                last_motor_left = motor_left.refer_vel;
            }
#ifndef MOTION_CONTROL
            UpdateBlink(num, led_state);
#endif
            break;
        case REF_MOTOR_RIGHT:
            motors[num].state = state;
            MOTOR_ENABLE2 = enable ^ motors[num].parameter_motor.enable_set;
            if(state == STATE_CONTROL_EMERGENCY) {
                last_motor_right = motor_right.refer_vel;
            }
#ifndef MOTION_CONTROL
            UpdateBlink(num, led_state);
#endif
            break;
    }
#ifdef MOTION_CONTROL
    UpdateBlink(LED1, led_state);
#endif
}

/* inline */ state_controller_t get_motor_state(short motIdx) {
    return motors[motIdx].state;
}

int MotorTaskController(void) {
    unsigned int t = TMR1; // Timing function
    volatile short i;
    /**
     * If high level control selected, then set new reference for all motors.
     */
    if (control_state != STATE_CONTROL_HIGH_DISABLE)
        HighLevelTaskController();

    for (i = 0; i < NUM_MOTORS; ++i) {
        switch (motors[i].state) {
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
                switch(i) {
                    case 0:
                        motor_left.refer_vel = 0;
                        break;
                    case 1:
                        motor_right.refer_vel = 0;
                        break;
                }
                break;
        }
        // Emergency controller
        if (motors[i].state > STATE_CONTROL_DISABLE) {
            if ((counter_alive[i] + 1) >= emergency.timeout) {
                /**
                 * Set Motor in emergency mode
                 */
                UpdateStateController(i, STATE_CONTROL_EMERGENCY);
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
                    motors[motIdx].k_mul = 2;
                    SwitchIcPrescaler(1, motIdx);
                }
                break;

            case IC_MODE1:
                if (vel0 < MIN1) {
                    motors[motIdx].k_mul = 1;
                    SwitchIcPrescaler(0, motIdx);
                } else if (vel0 >= MAX2) {
                    motors[motIdx].k_mul = 8;
                    SwitchIcPrescaler(2, motIdx);
                }
                break;

            case IC_MODE2:
                if (vel0 < MIN2) {
                    motors[motIdx].k_mul = 2;
                    SwitchIcPrescaler(1, motIdx);
                } else if (vel0 >= MAX3) {
                    motors[motIdx].k_mul = 32;
                    SwitchIcPrescaler(3, motIdx);
                }
                break;

            case IC_MODE3:
                if (vel0 < MIN3) {
                    motors[motIdx].k_mul = 8;
                    SwitchIcPrescaler(2, motIdx);
                }
                break;

            default:
                motors[motIdx].k_mul = 1;
                SwitchIcPrescaler(0, motIdx);
                break;
        }
    } else {
        int16_t vel1 = abs(motor_right.measure_vel);
        switch (IC2CONbits.ICM) {
            case IC_MODE0:
                if (vel1 >= MAX1) {
                    motors[motIdx].k_mul = 2;
                    SwitchIcPrescaler(1, motIdx);
                }
                break;

            case IC_MODE1:
                if (vel1 < MIN1) {
                    motors[motIdx].k_mul = 1;
                    SwitchIcPrescaler(0, motIdx);
                } else if (vel1 >= MAX2) {
                    motors[motIdx].k_mul = 8;
                    SwitchIcPrescaler(2, motIdx);
                }
                break;

            case IC_MODE2:
                if (vel1 < MIN2) {
                    motors[motIdx].k_mul = 2;
                    SwitchIcPrescaler(1, motIdx);
                } else if (vel1 >= MAX3) {
                    motors[motIdx].k_mul = 32;
                    SwitchIcPrescaler(3, motIdx);
                }
                break;

            case IC_MODE3:
                if (vel1 < MIN3) {
                    motors[motIdx].k_mul = 8;
                    SwitchIcPrescaler(2, motIdx);
                }
                break;

            default:
                motors[motIdx].k_mul = 1;
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

            motors[num].PulsEnc += (int) POS1CNT; // Odometry
            //int dAng = (int) POS1CNT * parameter_motor_left.k_vel * k_mul; // Odometry to angular
            POS1CNT = 0;
            // Speed calculation 
            if (SIG_VELtmp) {
                int16_t ic_contrib = SIG_VELtmp * ((motors[num].parameter_motor.k_vel * motors[num].k_mul) / timePeriodtmp);
                //int16_t odo_contrib = dAng* INV_PID_TIME; // TODO replace with real dT = 1/f_pid
                // motor_left.measure_vel = (ic_contrib + odo_contrib)/2;
                motor_left.measure_vel = ic_contrib;
            }
            //SelectIcPrescaler(0,motor_left.measure_vel);
            break;
        case REF_MOTOR_RIGHT:
            timePeriodtmp = timePeriodR;
            timePeriodR = 0;
            SIG_VELtmp = SIG_VELR;
            SIG_VELR = 0;
            motor_right.measure_vel = 0;

            motors[num].PulsEnc += (int) POS2CNT; // Odometry
            POS2CNT = 0;

            // Speed calculation
            if (SIG_VELtmp) {
                motor_right.measure_vel = SIG_VELtmp * (motors[num].parameter_motor.k_vel / (timePeriodtmp * motors[num].k_mul));
            }
            //SelectIcPrescaler(1,motor_right.measure_vel);
            break;
    }
}

int MotorPID(short num) {
    unsigned int t = TMR1; // Timing
    int pid_control;

    switch (num) {
        case REF_MOTOR_LEFT:
            motors[num].PIDstruct.controlReference = Q15(((float) motor_left.refer_vel) / constraint.max_left); // Setpoint
            motors[num].PIDstruct.measuredOutput = Q15(((float) motor_left.measure_vel) / constraint.max_left); // Measure

            PID(&motors[num].PIDstruct); // PID execution
            // Control value calculation
            motor_left.control_vel = motors[num].parameter_motor.versus * motors[num].PIDstruct.controlOutput;

            pid_control = (motor_left.control_vel >> 4) + 2048; // PWM value
            break;
        case REF_MOTOR_RIGHT:
            motors[num].PIDstruct.controlReference = Q15(((float) motor_right.refer_vel) / constraint.max_right); // Setpoint
            motors[num].PIDstruct.measuredOutput = Q15(((float) motor_right.measure_vel) / constraint.max_right); // Measure

            PID(&motors[num].PIDstruct); // PID execution
            // Control value calculation
            motor_right.control_vel = motors[num].parameter_motor.versus * motors[num].PIDstruct.controlOutput;

            pid_control = (motor_right.control_vel >> 4) + 2048; // PWM value

            break;
    }

    // PWM output
    SetDCMCPWM1(num+1, pid_control, 0);

    return TMR1 - t; // Execution time
}

bool Emergency(short num) {
    switch (num) {
        case REF_MOTOR_LEFT:
            motor_left.refer_vel -= (int16_t) (last_motor_left / emergency_step + 0.5f);
            if (SGN(motor_left.refer_vel) * last_motor_left < 0)
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
            motor_right.refer_vel -= (int16_t) (last_motor_right / emergency_step + 0.5f);
            if (SGN(motor_right.refer_vel) * last_motor_right < 0)
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
