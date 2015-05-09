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

#include <stdint.h>          /* For uint16_t definition                      */
#include <stdbool.h>         /* For true/false definition                    */
#include <dsp.h>
#include <pwm12.h>
#include "control/high_level_control.h"
#include "control/motors/motors.h"      /* variables/params used by motorsPID.c */
#include "system/system.h"
#include "system/user.h"
#include "packet/packet.h"

/**
 * Default value for motor parameters
 */
#define DEFAULT_VOLT_BRIDGE 12000
#define DEFAULT_CPR 300
#define DEFAULT_RATIO 30
#define DEFAULT_ENC_POSITION MOTOR_GEAR_ENC_BEFORE
#define DEFAULT_VERSUS_ROTATION MOTOR_ROTATION_COUNTERCLOCKWISE
#define DEFAULT_MOTOR_ENABLE MOTOR_ENABLE_LOW
/**
 * Default value for PID
 */
#define DEFAULT_KP 0.6
#define DEFAULT_KI 0.15
#define DEFAULT_KD 0.2

/*****************************************************************************/
/* Global Variable Declaration                                               */
/*****************************************************************************/
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

typedef struct new_motor {
    //Use ONLY in firmware
    //ICdata ICinfo; //Information for Input Capture
    pin_t * pin_enable;
    unsigned int CS_mask;
    uint8_t k_mul; // k_vel multiplier according to IC scale
    
    motor_t last_reference;
    unsigned int counter_alive;
    unsigned int counter_stop;
    int16_t pid_control;
    unsigned int counter_pid;
    /// Motor position
    volatile int PulsEnc;
    //Emergency
    float emergency_step;
    float emergency_stop;
    bool save_velocity;
    //gain motor
    float k_vel;
    float k_ang;
    //Common
    motor_emergency_t emergency;
    motor_parameter_t parameter_motor;
    motor_t constraint;
    motor_t reference;
    motor_t measure;
    //PID
    motor_pid_t pid;
    tPID PIDstruct;
    fractional kCoeffs[3]; //Coefficients KP, KI, KD
} new_motor_t;
new_motor_t motors[NUM_MOTORS];

/**/
// From interrupt
extern ICdata ICinfo[NUM_MOTORS];

//From system.c
extern process_t motor_process[PROCESS_MOTOR_LENGTH];

/*****************************************************************************/
/* User Functions                                                            */
/*****************************************************************************/

void init_motor(short motIdx) {
    motors[motIdx].measure.position = 0;
    motors[motIdx].measure.velocity = 0;
    motors[motIdx].measure.torque = 0;
    motors[motIdx].measure.volt = 0;
    motors[motIdx].measure.state = STATE_CONTROL_DISABLE;
    motors[motIdx].reference.position = 0;
    motors[motIdx].reference.velocity = 0;
    motors[motIdx].reference.torque = 0;
    motors[motIdx].reference.volt = 0;
    motors[motIdx].reference.state = STATE_CONTROL_DISABLE;
    //Counter frequency PID
    motors[motIdx].counter_pid = 0;
    //Input capture information
    ICinfo[motIdx].SIG_VEL = 0;
    ICinfo[motIdx].overTmr = 0;
    ICinfo[motIdx].timePeriod = 0;
    switch (motIdx) {
        case MOTOR_ZERO:
            motors[motIdx].pin_enable = &enable1;
            break;
        case MOTOR_ONE:
            motors[motIdx].pin_enable = &enable2;
            break;
    }
    motors[motIdx].CS_mask = 1 << motors[motIdx].pin_enable->CS_pin;
    motors[motIdx].k_mul = 1;
}

motor_parameter_t init_motor_parameters() {
    motor_parameter_t parameter;
    parameter.bridge.volt = DEFAULT_VOLT_BRIDGE;
    parameter.bridge.enable = DEFAULT_MOTOR_ENABLE;
    parameter.encoder.cpr = DEFAULT_CPR; //Gain to convert input capture value to velocity
    parameter.encoder.position = DEFAULT_ENC_POSITION;
    parameter.rotation = DEFAULT_VERSUS_ROTATION;
    parameter.ratio = (float) DEFAULT_RATIO; //Gain to convert QEI value to rotation movement

    return parameter;
}

/* inline */
motor_parameter_t get_motor_parameters(short motIdx) {
    return motors[motIdx].parameter_motor;
}

void update_motor_parameters(short motIdx, motor_parameter_t parameters) {
    //Update parameter configuration
    motors[motIdx].parameter_motor = parameters;
    // If CPR is before ratio
    //    ThC = CPR * RATIO   
    // else
    //    ThC = RATIO
    float angle_ratio;
    if(motors[motIdx].parameter_motor.encoder.position) {
        angle_ratio = motors[motIdx].parameter_motor.encoder.cpr * motors[motIdx].parameter_motor.ratio;
    } else {
        angle_ratio = motors[motIdx].parameter_motor.encoder.cpr;
    }
    //Start define with fixed K_vel conversion velocity
    // KVEL = FRTMR2 *  [ 2*pi / ( ThC * 2 ) ] * 1000 (velocity in milliradiant)
    motors[motIdx].k_vel = (float) 1000.0f * FRTMR2 * 2 * PI / (angle_ratio * 2);
    //Start define with fixed K_ang conversion angular
    // K_ANG = 2*PI / ( ThC * (QUADRATURE = 4) )
    motors[motIdx].k_ang = (float) 2*PI / (angle_ratio * 4);
    //Update encoder swap
    switch (motIdx) {
        case MOTOR_ZERO:
            QEI1CONbits.SWPAB = (motors[motIdx].parameter_motor.rotation >= 1) ? 1 : 0; // Phase A and Phase B inputs swapped
            break;
        case MOTOR_ONE:
            QEI2CONbits.SWPAB = (motors[motIdx].parameter_motor.rotation >= 1) ? 1 : 0; // Phase A and Phase B inputs swapped
            break;
    }
}

motor_t init_motor_constraints() {
    motor_t constraint;
    constraint.state = 0;
    constraint.position = -1;
    constraint.torque = -1;
    constraint.volt = -1;
    constraint.velocity = 25000;
    return constraint;
}

/* inline */
motor_t get_motor_constraints(short motIdx) {
    return motors[motIdx].constraint;
}

void update_motor_constraints(short motIdx, motor_t contraints) {
    //Update parameter constraints
    motors[motIdx].constraint = contraints;
}

motor_pid_t init_motor_pid() {
    motor_pid_t pid;
    pid.kp = DEFAULT_KP;
    pid.ki = DEFAULT_KI;
    pid.kd = DEFAULT_KD;
    return pid;
}

/* inline */
motor_pid_t get_motor_pid(short motIdx) {
    return motors[motIdx].pid;
}

void update_motor_pid(short motIdx, motor_pid_t pid) {
    // Update value of pid
    motors[motIdx].pid = pid;
    motors[motIdx].kCoeffs[0] = Q15(motors[motIdx].pid.kp);
    motors[motIdx].kCoeffs[1] = Q15(motors[motIdx].pid.ki);
    motors[motIdx].kCoeffs[2] = Q15(motors[motIdx].pid.kd);
    switch (motIdx) {
        case MOTOR_ZERO:
            //Initialize the PID data structure: PIDstruct
            //Set up pointer to derived coefficients
            motors[motIdx].PIDstruct.abcCoefficients = &abcCoefficient1[0];
            //Set up pointer to controller history samples
            motors[motIdx].PIDstruct.controlHistory = &controlHistory1[0];
            break;
        case MOTOR_ONE:
            //Initialize the PID data structure: PIDstruct
            //Set up pointer to derived coefficients
            motors[motIdx].PIDstruct.abcCoefficients = &abcCoefficient2[0];
            //Set up pointer to controller history samples
            motors[motIdx].PIDstruct.controlHistory = &controlHistory2[0];
            break;
    }
    // Clear the controller history and the controller output
    PIDInit(&motors[motIdx].PIDstruct);
    //Derive the a,b, & c coefficients from the Kp, Ki & Kd
    PIDCoeffCalc(&motors[motIdx].kCoeffs[0], &motors[motIdx].PIDstruct);
}
             
motor_emergency_t init_motor_emergency() {
    motor_emergency_t emergency;
    emergency.slope_time = 1.0;
    emergency.timeout = 500;
    emergency.bridge_off = 2.0;
    return emergency;
}

/* inline */
motor_emergency_t get_motor_emergency(short motIdx) {
    return motors[motIdx].emergency;
}

void update_motor_emergency(short motIdx, motor_emergency_t emergency_data) {
    motors[motIdx].emergency = emergency_data;
    motors[motIdx].emergency_step = motors[motIdx].emergency.slope_time * FRTMR1;
    motors[motIdx].emergency_stop = motors[motIdx].emergency.bridge_off * FRTMR1;
    motors[motIdx].counter_alive = 0;
    motors[motIdx].counter_stop = 0;
}

/* inline */
motor_t get_motor_measures(short motIdx) {
    motors[motIdx].measure.volt = motors[motIdx].pid_control / motors[motIdx].parameter_motor.bridge.volt;
    return motors[motIdx].measure;
}
             
/* inline */
motor_t get_motor_reference(short motIdx) {
    return motors[motIdx].reference;
}
             
/* inline */ void reset_motor_position_measure(short motIdx, float value) {
    motors[motIdx].measure.position = value;  
}
             
int set_motor_velocity(short motIdx, motor_control_t reference) {
    unsigned int t = TMR1; // Timing function
    motors[motIdx].counter_alive = 0; //Reset time emergency
    if(motors[motIdx].reference.state != STATE_CONTROL_VELOCITY) {
        set_motor_state(motIdx, STATE_CONTROL_VELOCITY);
    }
    motors[motIdx].reference.velocity = reference;
    if (abs(motors[motIdx].reference.velocity) > motors[motIdx].constraint.velocity) {
        motors[motIdx].reference.velocity = SGN(motors[motIdx].reference.velocity) * motors[motIdx].constraint.velocity;
    }
    return TMR1 - t; // Time of execution
}
    
/* inline */
motor_state_t get_motor_state(short motIdx) {
    return motors[motIdx].measure.state;
}

void set_motor_state(short motIdx, motor_state_t state) {
    volatile bool enable = (state != STATE_CONTROL_DISABLE) ? true : false;
    int led_state = (state != STATE_CONTROL_EMERGENCY) ? state + 1 : state;
    /**
     * Set enable or disable motors
     */

    switch (motIdx) {
        case -1:
            motors[MOTOR_ZERO].reference.state = state;
            motors[MOTOR_ONE].reference.state = state;
            MOTOR_ENABLE1_BIT = enable ^ motors[0].parameter_motor.bridge.enable;
            MOTOR_ENABLE2_BIT = enable ^ motors[1].parameter_motor.bridge.enable;
#ifndef MOTION_CONTROL
            UpdateBlink(0, led_state);
            UpdateBlink(1, led_state);
#endif
            break;
        case MOTOR_ZERO:
            motors[motIdx].reference.state = state;
            MOTOR_ENABLE1_BIT = enable ^ motors[motIdx].parameter_motor.bridge.enable;
            if (state == STATE_CONTROL_EMERGENCY) {
                motors[motIdx].last_reference.velocity = motors[motIdx].reference.velocity;
            }
#ifndef MOTION_CONTROL
            UpdateBlink(motIdx, led_state);
#endif
            break;
        case MOTOR_ONE:
            motors[motIdx].reference.state = state;
            MOTOR_ENABLE2_BIT = enable ^ motors[motIdx].parameter_motor.bridge.enable;
            if (state == STATE_CONTROL_EMERGENCY) {
                motors[motIdx].last_reference.velocity = motors[motIdx].reference.velocity;
            }
#ifndef MOTION_CONTROL
            UpdateBlink(motIdx, led_state);
#endif
            break;
    }
#ifdef MOTION_CONTROL
    UpdateBlink(LED1, led_state);
#endif
}

int MotorTaskController(void) {
    unsigned int t = TMR1; // Timing function
    volatile short i;

    for (i = 0; i < NUM_MOTORS; ++i) {
        switch (motors[i].reference.state) {
            case STATE_CONTROL_EMERGENCY:
                /**
                 * Set motor in emergency mode
                 */
                Emergency(i);
                // Run emergency control
                motor_process[i].time = MotorPID(i);
                break;
            case STATE_CONTROL_POSITION:
                //TODO to be implemented
                break;
            case STATE_CONTROL_VELOCITY:
                if (motors[i].counter_pid >= motor_process[i].frequency) {
                    motor_process[i].time = MotorPID(i);
                    motors[i].counter_pid = 0;
                }
                motors[i].counter_pid++;
                break;
            case STATE_CONTROL_TORQUE:
                //TODO to be implemented
                break;
            case STATE_CONTROL_DIRECT:
                //TODO To be implemented (Read issue #14)
                break;
            default:
                motors[i].pid_control = 0;
                break;
        }
        // Emergency controller
        if (motors[i].reference.state > STATE_CONTROL_DISABLE) {
            if ((motors[i].counter_alive + 1) >= motors[i].emergency.timeout) {
                /**
                 * Set Motor in emergency mode
                 */
                set_motor_state(i, STATE_CONTROL_EMERGENCY);
                motors[i].counter_stop = 0;
                motors[i].counter_alive = 0;
            } else
                motors[i].counter_alive++;
        }
    }

    return TMR1 - t; // Time of execution
}

int measureVelocity(short motIdx) {
    unsigned int t = TMR1; // Timing function
    unsigned long timePeriodtmp;
    int SIG_VELtmp;
    motors[motIdx].measure.velocity = 0;
    timePeriodtmp = ICinfo[motIdx].timePeriod;
    ICinfo[motIdx].timePeriod = 0;
    SIG_VELtmp = ICinfo[motIdx].SIG_VEL;
    ICinfo[motIdx].SIG_VEL = 0;
    // State control
    motors[motIdx].measure.state = motors[motIdx].reference.state;
    // Evaluate velocity
    if (SIG_VELtmp) {
        int16_t vel = SIG_VELtmp * (motors[motIdx].k_vel / timePeriodtmp);
        motors[motIdx].measure.velocity = vel;
    }
    //Evaluate position
    switch (motIdx) {
        case MOTOR_ZERO:
            motors[motIdx].PulsEnc += (int) POS1CNT;
            POS1CNT = 0;
            break;
        case MOTOR_ONE:
            motors[motIdx].PulsEnc += (int) POS2CNT;
            POS2CNT = 0;
            break;
    }
    // Evaluate angle position
    motors[motIdx].measure.position += motors[motIdx].PulsEnc * motors[motIdx].k_ang;
    if(abs(motors[motIdx].measure.position) > 2*PI) {
        motors[motIdx].measure.position = 0;
    }
    motors[motIdx].PulsEnc = 0;
    return TMR1 - t; // Time of execution
}

int MotorPID(short motIdx) {
    unsigned int t = TMR1; // Timing
    //Measure velocity
    measureVelocity(motIdx);
    // Setpoint
    motors[motIdx].PIDstruct.controlReference =  Q15(((float) motors[motIdx].reference.velocity) / motors[motIdx].constraint.velocity);
    // Measure
    motors[motIdx].PIDstruct.measuredOutput = Q15(((float) motors[motIdx].measure.velocity) / motors[motIdx].constraint.velocity);
    // PID execution
    PID(&motors[motIdx].PIDstruct);
    // Control value calculation
    motors[motIdx].pid_control = ((motors[motIdx].parameter_motor.rotation * motors[motIdx].PIDstruct.controlOutput) >> 4);
    // PWM output
    SetDCMCPWM1(motIdx + 1,  motors[motIdx].pid_control + 2048, 0);

    return TMR1 - t; // Execution time
}

bool Emergency(short motIdx) {
    motors[motIdx].reference.velocity -= motors[motIdx].last_reference.velocity / (int16_t) (motors[motIdx].emergency_step + 0.5f);
    if (SGN(motors[motIdx].reference.velocity) * motors[motIdx].last_reference.velocity < 0)
        motors[motIdx].reference.velocity = 0;
    if (motors[motIdx].reference.velocity == 0) {
        if ((motors[motIdx].counter_stop + 1) >= motors[motIdx].emergency_stop) {
            set_motor_state(motIdx, STATE_CONTROL_DISABLE);
            motors[motIdx].counter_stop = 0;
        } else
            motors[motIdx].counter_stop++;
    }
    return true;
}

void adc_motors_current(void) {
    int AdcCount = 0; //Counter
    long ADCValueTmp[ADC_CHANNELS] = {0, 0}; //Array to filter ADC data

    for (AdcCount = 0; AdcCount < ADC_BUFF; AdcCount++) // Evaluate mean value
    {
        ADCValueTmp[MOTOR_ZERO] += AdcBuffer[MOTOR_ZERO][AdcCount]; //Sum for AN0
        ADCValueTmp[MOTOR_ONE] += AdcBuffer[MOTOR_ONE][AdcCount]; //Sum for AN1
    }
    motors[MOTOR_ZERO].measure.torque = ADCValueTmp[MOTOR_ZERO] >> 6; //Shift
    motors[MOTOR_ONE].measure.torque = ADCValueTmp[MOTOR_ONE] >> 6; //Shift
}
