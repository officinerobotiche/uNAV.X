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
fractional abcCoefficient1[3] __attribute__((section(".xbss, bss, xmemory")));
fractional controlHistory1[3] __attribute__((section(".ybss, bss, ymemory")));
fractional abcCoefficient2[3] __attribute__((section(".xbss, bss, xmemory")));
fractional controlHistory2[3] __attribute__((section(".ybss, bss, ymemory")));

// ADC buffer, 2 channels (AN0, AN1), 32 bytes each, 2 x 32 = 64 bytes
int AdcBuffer[2][ADC_BUFF] __attribute__((space(dma), aligned(256)));
pin_t enable1 = {&MOTOR_ENABLE1_PORT, MOTOR_ENABLE1_NUM};
pin_t enable2 = {&MOTOR_ENABLE2_PORT, MOTOR_ENABLE2_NUM};

/** */

#define BUFFER_SIZE 16

typedef struct new_motor {
    //Use ONLY in firmware
    //ICdata ICinfo; //Information for Input Capture
    pin_t * pin_enable;
    unsigned int CS_mask;
    uint8_t k_mul; // k_vel multiplier according to IC scale
    volatile int PulsEnc; //Buffer for deadReckoning
    float last_velocity;
    unsigned int counter_alive;
    unsigned int counter_stop;
    //Common
    state_controller_t state;
    parameter_motor_t parameter_motor;
    motor_t motor;
    //PID
    pid_control_t pid;
    tPID PIDstruct;
    fractional kCoeffs[3]; //Coefficients KP, KI, KD
} new_motor_t;
new_motor_t motors[NUM_MOTORS];

#define NUM_VALUES 10
int16_t velocity[NUM_VALUES];

constraint_t constraint;

//variables for emergency
emergency_t emergency;
float emergency_step = 0;
float emergency_stop = 0;
bool save_velocity = true;

/**/
// From interrupt
extern ICdata ICinfo[NUM_MOTORS];

//From high_level_control
extern volatile unsigned int control_state;

//From System
extern parameter_system_t parameter_system;

/******************************************************************************/
/* User Functions                                                             */
/******************************************************************************/

void init_motor(short num) {
    motors[num].motor.control_vel = 0;
    motors[num].motor.measure_vel = 0;
    motors[num].motor.refer_vel = 0;
    motors[num].motor.current = 0;
    //Input capture information
    ICinfo[num].SIG_VEL = 0;
    ICinfo[num].overTmr = 0;
    ICinfo[num].timePeriod = 0;
    switch (num) {
        case REF_MOTOR_LEFT:
            motors[num].pin_enable = &enable1;
            constraint.max_left = 25000;
            break;
        case REF_MOTOR_RIGHT:
            motors[num].pin_enable = &enable2;
            constraint.max_right = 25000;
            break;
    }
    motors[num].CS_mask = 1 << motors[num].pin_enable->CS_pin;
    motors[num].k_mul = 1;
}

parameter_motor_t init_parameter_motors(short num) {
    parameter_motor_t parameter;
    parameter.k_vel = K_VEL; //Gain to convert input capture value to velocity
    parameter.k_ang = K_ANG; //Gain to convert QEI value to rotation movement
    parameter.versus = 1;
    parameter.enable_set = false;
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
    motors[num].kCoeffs[0] = Q15(motors[num].pid.kp);
    motors[num].kCoeffs[1] = Q15(motors[num].pid.ki);
    motors[num].kCoeffs[2] = Q15(motors[num].pid.kd);
    switch (num) {
        case REF_MOTOR_LEFT:
            //Initialize the PID data structure: PIDstruct
            //Set up pointer to derived coefficients
            motors[num].PIDstruct.abcCoefficients = &abcCoefficient1[0];
            //Set up pointer to controller history samples
            motors[num].PIDstruct.controlHistory = &controlHistory1[0];
            break;
        case REF_MOTOR_RIGHT:
            //Initialize the PID data structure: PIDstruct
            //Set up pointer to derived coefficients
            motors[num].PIDstruct.abcCoefficients = &abcCoefficient2[0];
            //Set up pointer to controller history samples
            motors[num].PIDstruct.controlHistory = &controlHistory2[0];
            break;
    }
    // Clear the controler history and the controller output
    PIDInit(&motors[num].PIDstruct);
    //Derive the a,b, & c coefficients from the Kp, Ki & Kd
    PIDCoeffCalc(&motors[num].kCoeffs[0], &motors[num].PIDstruct);
}

/* inline */ pid_control_t get_pid_value(short motIdx) {
    return motors[motIdx].pid;
}

emergency_t init_parameter_emergency(short num) {
    emergency_t emer;
    motors[num].counter_alive = 0;
    motors[num].counter_stop = 0;
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

int set_motor_velocity(short number, int16_t ref_velocity) {
    unsigned int t = TMR1; // Timing function
    motors[number].counter_alive = 0; //Reset time emergency
    motors[number].motor.refer_vel = ref_velocity;
    switch (number) {
        case REF_MOTOR_LEFT:
            if (abs(motors[number].motor.refer_vel) > constraint.max_left) {
                motors[number].motor.refer_vel = SGN(motors[number].motor.refer_vel) * constraint.max_left;
            }
            break;
        case REF_MOTOR_RIGHT:
            if (abs(motors[number].motor.refer_vel) > constraint.max_right) {
                motors[number].motor.refer_vel = SGN(motors[number].motor.refer_vel) * constraint.max_right;
            }
            break;
    }
    return TMR1 - t; // Time of esecution
}

/* inline */ motor_t get_motor_information(short motIdx) {
    return motors[motIdx].motor;
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
            MOTOR_ENABLE1_BIT = enable ^ motors[0].parameter_motor.enable_set;
            MOTOR_ENABLE2_BIT = enable ^ motors[1].parameter_motor.enable_set;
#ifndef MOTION_CONTROL
            UpdateBlink(0, led_state);
            UpdateBlink(1, led_state);
#endif
            break;
        case REF_MOTOR_LEFT:
            motors[num].state = state;
            MOTOR_ENABLE1_BIT = enable ^ motors[num].parameter_motor.enable_set;
            if(state == STATE_CONTROL_EMERGENCY) {
                motors[num].last_velocity = motors[num].motor.refer_vel;
            }
#ifndef MOTION_CONTROL
            UpdateBlink(num, led_state);
#endif
            break;
        case REF_MOTOR_RIGHT:
            motors[num].state = state;
            MOTOR_ENABLE2_BIT = enable ^ motors[num].parameter_motor.enable_set;
            if(state == STATE_CONTROL_EMERGENCY) {
                motors[num].last_velocity = motors[num].motor.refer_vel;
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
                // No other operations
                break;
            case STATE_CONTROL_TORQUE:
                //TODO to be implemented
                break;
            default:
                motors[i].motor.refer_vel = 0;
                break;
        }
        // Emergency controller
        if (motors[i].state > STATE_CONTROL_DISABLE) {
            if ((motors[i].counter_alive + 1) >= emergency.timeout) {
                /**
                 * Set Motor in emergency mode
                 */
                UpdateStateController(i, STATE_CONTROL_EMERGENCY);
                motors[i].counter_stop = 0;
                motors[i].counter_alive = 0;
            } else
                motors[i].counter_alive++;
        }
    }

    return TMR1 - t; // Time of esecution
}

int measureVelocity(short num) {
    unsigned int t = TMR1; // Timing function
    unsigned long timePeriodtmp;
    int SIG_VELtmp;
    motors[num].motor.measure_vel = 0;
    timePeriodtmp = ICinfo[num].timePeriod;
    ICinfo[num].timePeriod = 0;
    SIG_VELtmp = ICinfo[num].SIG_VEL;
    ICinfo[num].SIG_VEL = 0;
    // Evaluate velocity
    if (SIG_VELtmp) {
        int16_t vel = SIG_VELtmp * (motors[num].parameter_motor.k_vel / timePeriodtmp);
        motors[num].motor.measure_vel = vel;
    }
    
    //Evaluate position
    switch (num) {
        case REF_MOTOR_LEFT:
            motors[num].PulsEnc += (int) POS1CNT; // Odometry
            POS1CNT = 0;
            break;
        case REF_MOTOR_RIGHT:
            motors[num].PulsEnc += (int) POS2CNT; // Odometry
            POS2CNT = 0;
            break;
    }
    return TMR1 - t; // Time of esecution
}

int MotorPID(short num) {
    unsigned int t = TMR1; // Timing
    int pid_control;
    //Measure velocity
    measureVelocity(num);
    switch (num) {
        case REF_MOTOR_LEFT:
            motors[num].PIDstruct.controlReference = Q15(((float) motors[num].motor.refer_vel) / constraint.max_left); // Setpoint
            motors[num].PIDstruct.measuredOutput = Q15(((float) motors[num].motor.measure_vel) / constraint.max_left); // Measure
            break;
        case REF_MOTOR_RIGHT:
            motors[num].PIDstruct.controlReference = Q15(((float) motors[num].motor.refer_vel) / constraint.max_right); // Setpoint
            motors[num].PIDstruct.measuredOutput = Q15(((float) motors[num].motor.measure_vel) / constraint.max_right); // Measure
            break;
    }
    PID(&motors[num].PIDstruct); // PID execution
    // Control value calculation
    motors[num].motor.control_vel = motors[num].parameter_motor.versus * motors[num].PIDstruct.controlOutput;
    pid_control = (motors[num].motor.control_vel >> 4) + 2048; // PWM value

    // PWM output
    SetDCMCPWM1(num+1, pid_control, 0);

    return TMR1 - t; // Execution time
}

bool Emergency(short num) {
    motors[num].motor.refer_vel -= (int16_t) (motors[num].last_velocity / emergency_step + 0.5f);
    if (SGN(motors[num].motor.refer_vel) * motors[num].last_velocity < 0)
        motors[num].motor.refer_vel = 0;
    if (motors[num].motor.refer_vel == 0) {
        if ((motors[num].counter_stop + 1) >= emergency_stop) {
            UpdateStateController(num, STATE_CONTROL_DISABLE);
            motors[num].counter_stop = 0;
        } else
            motors[num].counter_stop++;
    }
    return true;
}

void adc_motors_current(void) {
    int AdcCount = 0; //Counter
    long ADCValueTmp[ADC_CHANNELS] = {0, 0}; //Array to filter ADC data

    for (AdcCount = 0; AdcCount < ADC_BUFF; AdcCount++) // Evaluate mean value
    {
        ADCValueTmp[REF_MOTOR_LEFT] += AdcBuffer[REF_MOTOR_LEFT][AdcCount]; //Sum for AN0
        ADCValueTmp[REF_MOTOR_RIGHT] += AdcBuffer[REF_MOTOR_RIGHT][AdcCount]; //Sum for AN1
    }
    motors[REF_MOTOR_LEFT].motor.current = ADCValueTmp[REF_MOTOR_LEFT] >> 6; //Shift
    motors[REF_MOTOR_RIGHT].motor.current = ADCValueTmp[REF_MOTOR_RIGHT] >> 6; //Shift
}
