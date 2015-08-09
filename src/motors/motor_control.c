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

#include <peripherals/gpio.h>
#include <system/task_manager.h>

#include <or_math/math.h>

#include "high_control/manager.h"
#include "motors/motor_control.h"      /* variables/params used by motorsPID.c */
#include "system/system.h"
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
#define DEFAULT_FREQ_MOTOR_MANAGER 1000 // In Herts
#define DEFAULT_FREQ_MOTOR_CONTROL_VELOCITY 1000 // In Herts
#define DEFAULT_FREQ_MOTOR_CONTROL_EMERGENCY 1000 // In Herts

#define NUMBER_CONTROL_FROM_ENUM(x) ( (x) + 1 )
#define NUMBER_CONTROL_FROM_ARRAY(x) ( (x) - 1 )

/*****************************************************************************/
/* Global Variable Declaration                                               */
/*****************************************************************************/

#define MOTOR "MOTOR"
static string_data_t _MODULE_MOTOR = {MOTOR, sizeof(MOTOR)};

/**
 * xc16 PID source in: folder_install_microchip_software/xc16/1.2x/src/libdsp.zip
 * on zip file: asm/pid.s
 */
fractional abcCoefficient1[3] __attribute__((section(".xbss, bss, xmemory")));
fractional controlHistory1[3] __attribute__((section(".ybss, bss, ymemory")));
fractional abcCoefficient2[3] __attribute__((section(".xbss, bss, xmemory")));
fractional controlHistory2[3] __attribute__((section(".ybss, bss, ymemory")));

/** */

#define NUM_CONTROLLERS 4

typedef struct _motor_firmware {
    //Use ONLY in firmware
    //ICdata ICinfo; //Information for Input Capture
    gpio_t* pin_enable;
    gp_peripheral_t* pin_current;
    gp_peripheral_t* pin_voltage;
    uint8_t k_mul; // k_vel multiplier according to IC scale
    motor_t last_reference;
    unsigned int counter_alive;
    unsigned int counter_stop;
    /// event register
    hEvent_t task_manager;
    task_t controllers[NUM_CONTROLLERS];
    /// Motor position
    uint32_t angle_ratio;
    volatile int PulsEnc;
    volatile int32_t enc_angle;
    //Emergency
    float emergency_step;
    float emergency_stop;
    bool save_velocity;
    //gain motor
    float k_vel;
    float k_ang;
    //Common
    motor_diagnostic_t diagnostic;
    motor_emergency_t emergency;
    motor_parameter_t parameter_motor;
    motor_t constraint;
    motor_t reference;
    motor_t measure;
    //PID
    motor_pid_t pid;
    tPID PIDstruct;
    fractional kCoeffs[3]; //Coefficients KP, KI, KD
} motor_firmware_t;
motor_firmware_t motors[NUM_MOTORS];

/**/
// From interrupt
extern ICdata ICinfo[NUM_MOTORS];

/*****************************************************************************/
/* User Functions                                                            */
/*****************************************************************************/

void reset_motor_data(motor_t* motor) {
    motor->position_delta = 0;
    motor->position = 0;
    motor->velocity = 0;
    motor->torque = 0;
    motor->volt = 0;
    motor->state = CONTROL_DISABLE;
}

void init_controllers(task_t* controllers) {
    int i;
    for(i = 0; i < NUM_CONTROLLERS; ++i) {
        controllers[i].task = NULL;
        controllers[i].frequency = 1;
    }
}

void init_motor(const short motIdx, gpio_t* enable, gp_peripheral_t* current, gp_peripheral_t* voltage) {
    reset_motor_data(&motors[motIdx].measure);
    reset_motor_data(&motors[motIdx].reference);
    init_controllers(motors[motIdx].controllers);
    //Setup diagnostic
    motors[motIdx].diagnostic.current = 0;
    motors[motIdx].diagnostic.temperature = 0;
    //Input capture information
    ICinfo[motIdx].SIG_VEL = 0;
    ICinfo[motIdx].overTmr = 0;
    ICinfo[motIdx].timePeriod = 0;
    /// Setup bit enable
    motors[motIdx].pin_enable = enable;
    gpio_register(motors[motIdx].pin_enable);
    /// Setup ADC current and temperature
    motors[motIdx].pin_current = current;
    motors[motIdx].pin_current->gpio.type = GPIO_ANALOG;
    gpio_register_peripheral(motors[motIdx].pin_current);
    motors[motIdx].pin_voltage = voltage;
    motors[motIdx].pin_voltage->gpio.type = GPIO_ANALOG;
    gpio_register_peripheral(motors[motIdx].pin_voltage);
    
    motors[motIdx].k_mul = 1;
    
    /// Register event and add in task controller - Working at 1KHz
    motors[motIdx].task_manager = task_load_data(register_event_p(register_module(&_MODULE_MOTOR), &MotorTaskController, EVENT_PRIORITY_MEDIUM), 
                                    DEFAULT_FREQ_MOTOR_MANAGER, 1, (char) motIdx);
    /// Run task controller
    task_set(motors[motIdx].task_manager, RUN);
    /// Load controller EMERGENCY - Working at 1KHz
    motors[motIdx].controllers[NUMBER_CONTROL_FROM_ENUM(CONTROL_EMERGENCY)].frequency = DEFAULT_FREQ_MOTOR_CONTROL_EMERGENCY;
    motors[motIdx].controllers[NUMBER_CONTROL_FROM_ENUM(CONTROL_EMERGENCY)].task = task_load_data(register_event_p(register_module(&_MODULE_MOTOR), &Emergency, EVENT_PRIORITY_HIGH),
            motors[motIdx].controllers[NUMBER_CONTROL_FROM_ENUM(CONTROL_EMERGENCY)].frequency, 1, (char) motIdx);
    /// Load controllers VELOCITY - Working at 1KHz
    motors[motIdx].controllers[NUMBER_CONTROL_FROM_ENUM(CONTROL_VELOCITY)].frequency = DEFAULT_FREQ_MOTOR_CONTROL_VELOCITY;
    motors[motIdx].controllers[NUMBER_CONTROL_FROM_ENUM(CONTROL_VELOCITY)].task = task_load_data(register_event_p(register_module(&_MODULE_MOTOR), &controller, EVENT_PRIORITY_MEDIUM),
            motors[motIdx].controllers[NUMBER_CONTROL_FROM_ENUM(CONTROL_VELOCITY)].frequency, 1, (char) motIdx);
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

inline motor_parameter_t get_motor_parameters(short motIdx) {
    return motors[motIdx].parameter_motor;
}

void update_motor_parameters(short motIdx, motor_parameter_t parameters) {
    //Update parameter configuration
    motors[motIdx].parameter_motor = parameters;
    // If CPR is before ratio
    //    ThC = CPR * RATIO   
    // else
    //    ThC = RATIO
    if(motors[motIdx].parameter_motor.encoder.position) {
        motors[motIdx].angle_ratio = motors[motIdx].parameter_motor.encoder.cpr * ((uint32_t) 1000 * motors[motIdx].parameter_motor.ratio);
    } else {
        motors[motIdx].angle_ratio = (uint32_t) 1000 * motors[motIdx].parameter_motor.encoder.cpr;
    }
    //Start define with fixed K_vel conversion velocity
    // KVEL = FRTMR2 *  [ 2*pi / ( ThC * 2 ) ] * 1000 (velocity in milliradiant)
    motors[motIdx].k_vel = (float) 1000000.0f * FRTMR2 * 2 * PI / (motors[motIdx].angle_ratio * 2);
    //Start define with fixed K_ang conversion angular
    // K_ANG = 2*PI / ( ThC * (QUADRATURE = 4) )
    motors[motIdx].k_ang = (float) 2000.0f *PI / (motors[motIdx].angle_ratio * 4);
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

inline motor_t get_motor_constraints(short motIdx) {
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

inline motor_pid_t get_motor_pid(short motIdx) {
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

inline motor_emergency_t get_motor_emergency(short motIdx) {
    return motors[motIdx].emergency;
}

void update_motor_emergency(short motIdx, motor_emergency_t emergency_data) {
    motors[motIdx].emergency = emergency_data;
    motors[motIdx].emergency_step = motors[motIdx].emergency.slope_time * FRTMR1;
    motors[motIdx].emergency_stop = motors[motIdx].emergency.bridge_off * FRTMR1;
    motors[motIdx].counter_alive = 0;
    motors[motIdx].counter_stop = 0;
}

inline motor_t get_motor_measures(short motIdx) {
    motors[motIdx].measure.position_delta = motors[motIdx].k_ang * motors[motIdx].PulsEnc;
    motors[motIdx].measure.position = motors[motIdx].enc_angle * motors[motIdx].k_ang; 
    motors[motIdx].measure.volt = motors[motIdx].reference.volt / motors[motIdx].parameter_motor.bridge.volt;
    motors[motIdx].measure.torque = motors[motIdx].diagnostic.current; //TODO Add a coefficient conversion
    motors[motIdx].PulsEnc = 0;
    return motors[motIdx].measure;
}

inline motor_t get_motor_reference(short motIdx) {
    return motors[motIdx].reference;
}
             
inline void reset_motor_position_measure(short motIdx, motor_control_t value) {
    motors[motIdx].measure.position = (float) value;  
}
             
int set_motor_reference(short motIdx, motor_state_t state, motor_control_t reference) {
    unsigned int t = TMR1; // Timing function
    if(state == CONTROL_VELOCITY) {
        motors[motIdx].counter_alive = 0; //Reset time emergency
        if(motors[motIdx].reference.state != CONTROL_VELOCITY) {
            set_motor_state(motIdx, CONTROL_VELOCITY);
        }
        motors[motIdx].reference.velocity = reference;
        if (abs(motors[motIdx].reference.velocity) > motors[motIdx].constraint.velocity) {
            motors[motIdx].reference.velocity = SGN(motors[motIdx].reference.velocity) * motors[motIdx].constraint.velocity;
        }
    }
    return TMR1 - t; // Time of execution
}
    
inline motor_state_t get_motor_state(short motIdx) {
    return motors[motIdx].measure.state;
}

void set_motor_state(short motIdx, motor_state_t state) {
    volatile bool enable = (state != CONTROL_DISABLE) ? true : false;
    int led_state = (state != CONTROL_EMERGENCY) ? state + 1 : state;
    
    /// Set enable or disable motors
    motors[motIdx].reference.state = state;
    if(enable ^ motors[motIdx].parameter_motor.bridge.enable)
        REGISTER_MASK_SET_HIGH(motors[motIdx].pin_enable->CS_PORT, motors[motIdx].pin_enable->CS_mask);
    else
        REGISTER_MASK_SET_LOW(motors[motIdx].pin_enable->CS_PORT, motors[motIdx].pin_enable->CS_mask);
    
    if (state == CONTROL_EMERGENCY) {
        motors[motIdx].last_reference.velocity = motors[motIdx].reference.velocity;
    }
    /// Disable PWM generator
    if (enable) {
        PTCONbits.PTEN = 1;
    } else {
        if ((motors[MOTOR_ZERO].reference.state == CONTROL_DISABLE) && 
                (motors[MOTOR_ONE].reference.state == CONTROL_DISABLE)) {
            PTCONbits.PTEN = 0;
        }
    }
#ifndef MOTION_CONTROL
    UpdateBlink(motIdx, led_state);
#elif MOTION_CONTROL
    UpdateBlink(LED1, led_state);
#endif
}

void MotorTaskController(int argc, int *argv) {
    
    short motIdx = (short) argv[0];
    /// Add new task controller
    if(motors[motIdx].reference.state != motors[motIdx].measure.state) {
        if(motors[motIdx].measure.state != CONTROL_DISABLE) {
            /// Stop old controller
            task_set(motors[motIdx].controllers[NUMBER_CONTROL_FROM_ENUM(motors[motIdx].measure.state)].task, STOP);
        }
        if(motors[motIdx].reference.state != CONTROL_DISABLE) {
            /// Load new controller
            if(motors[motIdx].controllers[NUMBER_CONTROL_FROM_ENUM(motors[motIdx].reference.state)].task != NULL) {
                /// Run controller in task manager
                task_set(motors[motIdx].controllers[NUMBER_CONTROL_FROM_ENUM(motors[motIdx].reference.state)].task, RUN);
            }
        } else if (motors[motIdx].reference.state == CONTROL_DISABLE) {
            /// Set PWM 0
            Motor_PWM(motIdx, 0);
        }
        /// Save new state controller
        motors[motIdx].measure.state = motors[motIdx].reference.state;
    }
    /// Check for emergency mode
    if(motors[motIdx].reference.state > CONTROL_DISABLE) {
        if ((motors[motIdx].counter_alive + 1) >= motors[motIdx].emergency.timeout) {
            /// Set Motor in emergency mode
            set_motor_state(motIdx, CONTROL_EMERGENCY);
            motors[motIdx].counter_stop = 0;
            motors[motIdx].counter_alive = 0;
        } else
            motors[motIdx].counter_alive++;
    }
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
    // Evaluate velocity
    if (SIG_VELtmp) {
        int16_t vel = SIG_VELtmp * (motors[motIdx].k_vel / timePeriodtmp);
        motors[motIdx].measure.velocity = vel;
    }
    //Evaluate position
    switch (motIdx) {
        case MOTOR_ZERO:
            motors[motIdx].PulsEnc += (int) POS1CNT;
            motors[motIdx].enc_angle += (int) POS1CNT;
            POS1CNT = 0;
            break;
        case MOTOR_ONE:
            motors[motIdx].PulsEnc += (int) POS2CNT;
            motors[motIdx].enc_angle +=  (int) POS2CNT;
            POS2CNT = 0;
            break;
    }
    // Evaluate angle position
    if(abs(motors[motIdx].enc_angle) > motors[motIdx].angle_ratio ) {
        motors[motIdx].enc_angle = 0;
    }
    return TMR1 - t; // Time of execution
}

void controller(int argc, int *argv) {
    
    short motIdx = (short) argv[0];
   // PWM output
    Motor_PWM(motIdx, MotorPID(motIdx));
}

inline void Motor_PWM(short motIdx, int pwm_control) {
   // PWM output
    motors[motIdx].reference.volt = pwm_control;
    SetDCMCPWM1(motIdx + 1, pwm_control + 2048, 0);
}

inline int MotorPID(short motIdx) {
    //Measure velocity
    measureVelocity(motIdx);
    // Setpoint
    motors[motIdx].PIDstruct.controlReference =  Q15(((float) motors[motIdx].reference.velocity) / motors[motIdx].constraint.velocity);
    // Measure
    motors[motIdx].PIDstruct.measuredOutput = Q15(((float) motors[motIdx].measure.velocity) / motors[motIdx].constraint.velocity);
    // PID execution
    PID(&motors[motIdx].PIDstruct);
    // Control value calculation
    return motors[motIdx].parameter_motor.rotation * (motors[motIdx].PIDstruct.controlOutput >> 4);
}

void Emergency(int argc, int *argv) {
    short motIdx = (short) argv[0];
    if (motors[motIdx].reference.velocity != 0) {
        motors[motIdx].reference.velocity -= motors[motIdx].last_reference.velocity / (int16_t) (motors[motIdx].emergency_step + 0.5f);
        if (SGN(motors[motIdx].reference.velocity) * motors[motIdx].last_reference.velocity < 0) {
            motors[motIdx].reference.velocity = 0;
        }
        // Velocity control output
        Motor_PWM(motIdx, MotorPID(motIdx));
    } else if (motors[motIdx].reference.velocity == 0) {
        if ((motors[motIdx].counter_stop + 1) >= motors[motIdx].emergency_stop) {
            set_motor_state(motIdx, CONTROL_DISABLE);
            motors[motIdx].counter_stop = 0;
        } else
            motors[motIdx].counter_stop++;
    }
}

//inline void adc_motors_current(ADC* AdcBuffer, size_t len) {
//    int AdcCount = 0; //Counter
//    long ADCValueTmp[NUM_MOTORS] = {0, 0}; //Array to filter ADC data
//
//    for (AdcCount = 0; AdcCount < len; ++AdcCount) // Evaluate mean value
//    {
//        ADCValueTmp[MOTOR_ZERO] += (*AdcBuffer)[MOTOR_ZERO][AdcCount]; //Sum for AN0
//        ADCValueTmp[MOTOR_ONE] += (*AdcBuffer)[MOTOR_ONE][AdcCount]; //Sum for AN1
//    }
//    motors[MOTOR_ZERO].diagnostic.current = ADCValueTmp[MOTOR_ZERO] >> 6; //Shift
//    motors[MOTOR_ONE].diagnostic.current = ADCValueTmp[MOTOR_ONE] >> 6; //Shift
//}
