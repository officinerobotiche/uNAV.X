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

#include <xc.h>              /* Device header file */

#include <stdint.h>          /* For uint16_t definition                      */
#include <stdbool.h>         /* For true/false definition                    */
#include <dsp.h>
#include <pwm12.h>

#include <peripherals/gpio.h>
#include <system/task_manager.h>
#include <system/soft_timer.h>

#include <or_math/math.h>
#include <or_math/statistics.h>

#include "high_control/manager.h"
#include "motors/motor_control.h"      /* variables/params used by motorsPID.c */

#include "system/system.h"
#include "packet/packet.h"

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

#define DEFAULT_FREQ_MOTOR_MANAGER 1000.0             // Task Manager 10Khz
#define DEFAULT_FREQ_MOTOR_CONTROL_EMERGENCY 1000.0   // In Herts
#define DEFAULT_FREQ_MOTOR_CONTROL_SAFETY 1000.0      // In Herts

/*****************************************************************************/
/* Global Variable Declaration                                               */
/*****************************************************************************/

#define MOTOR "MOTOR"
static string_data_t _MODULE_MOTOR = {MOTOR, sizeof(MOTOR)};

#define NUM_CONTROLLERS 3
// Get the number in controller array from enum_state_t
#define GET_CONTROLLER_NUM(X) ((X) - 1)
/**
 * xc16 PID source in: folder_install_microchip_software/xc16/1.2x/src/libdsp.zip
 * on zip file: asm/pid.s
 */
fractional abcCoefficient[NUM_MOTORS][NUM_CONTROLLERS][3] __attribute__((section(".xbss, bss, xmemory")));
fractional controlHistory[NUM_MOTORS][NUM_CONTROLLERS][3] __attribute__((section(".ybss, bss, ymemory")));

/** */

#define DEFAULT_PWM_OFFSET 2048
#define DEFAULT_PWM_MAX 2047
#define DEFAULT_PWM_MIN -2048

typedef struct _analog {
    int32_t gain;
    int32_t offset;
} analog_t;

/**
 * Definition of PID ecosystem
 */
typedef struct _pid_control {
    // Message information about value of PID
    motor_pid_t pid;
    // Struct of PID from dsp library
    tPID PIDstruct;
    // Coefficients KP, KI, KD
    fractional kCoeffs[3];
    // Coefficient K anti wind up
    fractional k_aw;
    // soft timer to launch PID controller
    soft_timer_t timer;
    // enable
    volatile bool enable;
    // anti wind up correction
    volatile fractional saturation;
} pid_controller_t;

/**
 * Use ONLY in firmware
 */
typedef struct _motor_firmware {
    // Information for Input Capture
    ICdata* ICinfo;
    // Enable pin for H-bridge
    gpio_t* pin_enable;
    int pin_current, pin_voltage;
    event_prescaler_t prescaler_callback;
    // Frequency manager;
    frequency_t manager_freq;
    volatile int pwm_limit;
    /// Task register
    hEvent_t motor_manager_event;
    hTask_t task_manager;
    hTask_t task_emergency;
    /// Motor position
    uint32_t angle_limit;
    volatile int PulsEnc;
    volatile int32_t enc_angle;
    //Emergency
    struct {
        soft_timer_t alive;
        soft_timer_t stop;
        uint16_t step;
    } motor_emergency;
    motor_emergency_t emergency;
    bool save_velocity;
    motor_control_t last_reference;
    // Safety
    struct {
        hTask_t task_safety;
        soft_timer_t stop;
        soft_timer_t restore;
        motor_state_t old_state;
        uint16_t step;
    } motor_safety;
    motor_safety_t safety;
    // Parameter and diagnostic
    motor_diagnostic_t diagnostic;
    motor_parameter_t parameter_motor;
    //gain motor
    int32_t k_vel_ic, k_vel_qei;
    float k_ang;
    // Velocity mean
    statistic_buffer mean_vel;
    //Internal value volt and current
    analog_t volt, current;
    //PID
    motor_state_t state;
    fractional pwm_saturation;
    volatile motor_control_t external_reference;
    volatile motor_control_t control_output;
    pid_controller_t controller[NUM_CONTROLLERS];
    //Common
    motor_t constraint;
    volatile motor_t controlOut;
    volatile motor_t reference;
    volatile motor_t measure;
} motor_firmware_t;
motor_firmware_t motors[NUM_MOTORS];

/*****************************************************************************/
/* User Functions                                                            */
/*****************************************************************************/

void reset_motor_data(volatile motor_t* motor) {
    motor->position_delta = 0;
    motor->position = 0;
    motor->velocity = 0;
    motor->current = 0;
    motor->effort = 0;
    motor->pwm = 0;
}

void initialize_controllers(short motIdx) {
    int i;
    // Initialize all controllers
    for(i = 0; i < NUM_CONTROLLERS; i++) {
        //Initialize the PID data structure: PIDstruct
        //Set up pointer to derived coefficients
        motors[motIdx].controller[i].PIDstruct.abcCoefficients = &abcCoefficient[motIdx][i][0];
        //Set up pointer to controller history samples
        motors[motIdx].controller[i].PIDstruct.controlHistory = &controlHistory[motIdx][i][0];
        // Clear the controller history and the controller output
        PIDInit(&motors[motIdx].controller[i].PIDstruct);
        // Enable
        motors[motIdx].controller[i].enable = false;
        // Saturation value
        motors[motIdx].controller[i].saturation = 0;
        // Gain anti wind up
        motors[motIdx].controller[i].k_aw = 0;
    }
}

hTask_t init_motor(const short motIdx, gpio_t* enable_, ICdata* ICinfo_, event_prescaler_t prescaler_event, int current_, int voltage_) {
    reset_motor_data(&motors[motIdx].measure);
    reset_motor_data(&motors[motIdx].reference);
    reset_motor_data(&motors[motIdx].controlOut);
    
    // PWM limit
    motors[motIdx].pwm_limit = DEFAULT_PWM_MAX;
    
    motors[motIdx].control_output = 0;
    motors[motIdx].external_reference = 0;
    motors[motIdx].pwm_saturation = 0;
    //Initialize controllers
    initialize_controllers(motIdx);

    motors[motIdx].prescaler_callback = prescaler_event;
    // Setup frequency task manager
    motors[motIdx].manager_freq = DEFAULT_FREQ_MOTOR_MANAGER;
    //Setup diagnostic
    motors[motIdx].diagnostic.time_control = 0;
    motors[motIdx].diagnostic.watt = 0;
    motors[motIdx].diagnostic.temperature = 0;
    motors[motIdx].diagnostic.time_control = 0;
    motors[motIdx].diagnostic.state = CONTROL_DISABLE;
    // Internal state
    motors[motIdx].state = CONTROL_DISABLE;
    // Register input capture
    motors[motIdx].ICinfo = ICinfo_;
    /// Setup bit enable
    motors[motIdx].pin_enable = enable_;
    gpio_register(motors[motIdx].pin_enable);
    /// Setup ADC current and temperature
    motors[motIdx].pin_current = current_;
    motors[motIdx].pin_voltage = voltage_;
    // Init mean buffer
    init_statistic_buffer(&motors[motIdx].mean_vel);
    /// Register event and add in task controller - Working at 1KHz
    hModule_t motor_manager_module = register_module(&_MODULE_MOTOR);
    motors[motIdx].motor_manager_event = register_event_p(motor_manager_module, &MotorTaskController, EVENT_PRIORITY_MEDIUM);
    motors[motIdx].task_manager = task_load_data(motors[motIdx].motor_manager_event, motors[motIdx].manager_freq, 1, (char) motIdx);
    /// Load controller EMERGENCY - Working at 1KHz
    hModule_t emegency_module = register_module(&_MODULE_MOTOR);
    hEvent_t emergency_event = register_event_p(emegency_module, &Emergency, EVENT_PRIORITY_HIGH);
    motors[motIdx].task_emergency = task_load_data(emergency_event, DEFAULT_FREQ_MOTOR_CONTROL_EMERGENCY, 1, (char) motIdx);
    /// Load controller SAFETY - Working at 1KHz
    hModule_t safety_module = register_module(&_MODULE_MOTOR);
    hEvent_t safety_event = register_event_p(safety_module, &Safety, EVENT_PRIORITY_HIGH);
    motors[motIdx].motor_safety.task_safety = task_load_data(safety_event, DEFAULT_FREQ_MOTOR_CONTROL_SAFETY, 1, (char) motIdx);
    // Return Task manager
    return motors[motIdx].task_manager;
}

motor_parameter_t init_motor_parameters() {
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

inline motor_parameter_t get_motor_parameters(short motIdx) {
    return motors[motIdx].parameter_motor;
}

void update_motor_parameters(short motIdx, motor_parameter_t parameters) {
    //Update parameter configuration
    memcpy(&motors[motIdx].parameter_motor, &parameters, sizeof(motor_parameter_t));
    // If CPR is before ratio
    //    ThC = CPR * RATIO   
    // else
    //    ThC = RATIO
    uint32_t angle_ratio;
    if(motors[motIdx].parameter_motor.encoder.type.position) {
        angle_ratio = motors[motIdx].parameter_motor.encoder.cpr * ((uint32_t) 1000 * motors[motIdx].parameter_motor.ratio);
    } else {
        angle_ratio = (uint32_t) 1000 * motors[motIdx].parameter_motor.encoder.cpr;
    }
    // Evaluate angle limit
    motors[motIdx].angle_limit = (angle_ratio * 4) / 1000;
    //Start define with fixed K_vel conversion velocity
    // KVEL = FRTMR2 *  [ 2*pi / ( ThC * 2 ) ] * 1000 (velocity in milliradiant)
    motors[motIdx].k_vel_ic = 1000000.0f * FRTMR2 * 2 * PI / (angle_ratio * 2);
    //Start define with fixed K_ang conversion angular
    // K_ANG = 2*PI / ( ThC * (QUADRATURE = 4) )
    motors[motIdx].k_ang = (float) 2000.0f *PI / (angle_ratio * 4);
    // vel_qei = 1000 * QEICNT * Kang / Tc = 1000 * QEICNT * Kang * Fc
    //motors[motIdx].k_vel_qei = (motors[motIdx].k_ang * 1000.0f * motors[motIdx].manager_freq);
    motors[motIdx].k_vel_qei = (motors[motIdx].k_ang * 1000.0f * ((float) motors[motIdx].controller[GET_CONTROLLER_NUM(CONTROL_VELOCITY)].pid.frequency));
    
    //Update encoder swap
    switch (motIdx) {
        case MOTOR_ZERO:
            QEI1CONbits.SWPAB = (motors[motIdx].parameter_motor.rotation >= 1) ? 1 : 0; // Phase A and Phase B inputs swapped
            break;
        case MOTOR_ONE:
            QEI2CONbits.SWPAB = (motors[motIdx].parameter_motor.rotation >= 1) ? 1 : 0; // Phase A and Phase B inputs swapped
            break;
    }
    // Convert gain current in [mA]
    motors[motIdx].current.gain = ( 1000.0 * GAIN_ADC ) / motors[motIdx].parameter_motor.bridge.current_gain + 0.5f;
    motors[motIdx].current.offset = (1000.0 * motors[motIdx].parameter_motor.bridge.current_offset ) / motors[motIdx].parameter_motor.bridge.current_gain + 0.5f;
    // Convert gain volt in [mV]
    motors[motIdx].volt.gain = 1000.0 * motors[motIdx].parameter_motor.bridge.volt_gain * GAIN_ADC;
    motors[motIdx].volt.offset = 1000.0 * motors[motIdx].parameter_motor.bridge.volt_offset;
    // Setup state with new bridge configuration
    set_motor_state(motIdx, motors[motIdx].state);
}

motor_t init_motor_constraints() {
    motor_t constraint;
    constraint.pwm = MOTOR_CONTROL_MAX;
    constraint.effort = MOTOR_CONTROL_MAX;
    constraint.current = MOTOR_CONTROL_MAX;
    constraint.velocity = MOTOR_CONTROL_MAX;
    constraint.position = MOTOR_CONTROL_MAX;
    return constraint;
}

inline motor_t get_motor_constraints(short motIdx) {
    return motors[motIdx].constraint;
}

void update_motor_constraints(short motIdx, motor_t constraints) {
    //Update parameter constraints
    memcpy(&motors[motIdx].constraint, &constraints, sizeof(motor_t));
    //Update PWM max value
    //Shifted from maximum motor_control_t value to maximum PWM value
    // 31bit -> 11 bit = 20
    motors[motIdx].pwm_limit = motors[motIdx].constraint.pwm >> 20;
}

inline motor_pid_t get_motor_pid(short motIdx, motor_state_t state) {
    int num_control = GET_CONTROLLER_NUM(state);
    return motors[motIdx].controller[num_control].pid;
}

bool update_motor_pid(short motIdx, motor_state_t state, motor_pid_t pid) {
    // If the motor control is the same in action you can not disable
    if(motors[motIdx].diagnostic.state == state && pid.enable == false) {
        return false;
    }
    // Check gains value
    // Check1 = | Kp + ki + kd | < 1 = INT16_MAX
    // Check2 = | -(Kp + 2*Kd) | < 1 = INT16_MAX
    // Check3 =      | Kd |      < 1 = INT16_MAX
    long check1 = labs(1000.0 * (pid.kp + pid.ki + pid.kd));
    long check2 = labs(1000.0 * (pid.kp + 2 * pid.kd));
    long check3 = labs(1000.0 * pid.kd);
    
    if(check1 < INT16_MAX && check2 < INT16_MAX && check3 < INT16_MAX) {
        int num_control = GET_CONTROLLER_NUM(state);
        // Update PID struct
        memcpy(&motors[motIdx].controller[num_control].pid, &pid, sizeof(motor_pid_t));
        // Write new coefficients
        motors[motIdx].controller[num_control].kCoeffs[0] = (int) (pid.kp * 1000.0);
        motors[motIdx].controller[num_control].kCoeffs[1] = (int) (pid.ki * 1000.0);
        motors[motIdx].controller[num_control].kCoeffs[2] = (int) (pid.kd * 1000.0);
        // Derive the a, b and c coefficients from the Kp, Ki & Kd
        PIDCoeffCalc(&motors[motIdx].controller[num_control].kCoeffs[0], 
                &motors[motIdx].controller[num_control].PIDstruct);
        // Clear the controller history and the controller output
        PIDInit(&motors[motIdx].controller[num_control].PIDstruct);
        // Gain anti wind up
        motors[motIdx].controller[num_control].k_aw = (int) (pid.kaw * 1000.0);
#ifdef CURRENT_CONTROL_IN_ADC_LOOP
        if(state == CONTROL_CURRENT) {
            // Manual reset frequency  current loop
            motors[motIdx].controller[num_control].pid.frequency = CURRENT_ADC_LOOP_FRQ;
        } else {
            // Initialize soft timer
            init_soft_timer(&motors[motIdx].controller[num_control].timer, motors[motIdx].manager_freq, 1000000 / pid.frequency);
        }
#else
        // Initialize soft timer
        init_soft_timer(&motors[motIdx].controller[num_control].timer, motors[motIdx].manager_freq, 1000000 / pid.frequency);
#endif
        // Set enable PID
        motors[motIdx].controller[num_control].enable = pid.enable;
        // Update K_qei
        if(state == CONTROL_VELOCITY) {
            motors[motIdx].k_vel_qei = motors[motIdx].k_ang * 1000.0f *((float) pid.frequency);
        }
        return true;
    } else
        return false;
}

inline motor_emergency_t get_motor_emergency(short motIdx) {
    return motors[motIdx].emergency;
}

void update_motor_emergency(short motIdx, motor_emergency_t emergency_data) {
    // Store emergency data
    memcpy(&motors[motIdx].emergency, &emergency_data, sizeof(motor_emergency_t));
    // Reset counter alive reference message
    init_soft_timer(&motors[motIdx].motor_emergency.alive, (motors[motIdx].manager_freq / 1000), emergency_data.timeout * 1000000);
    // Reset counter Emergency stop
    init_soft_timer(&motors[motIdx].motor_emergency.stop, motors[motIdx].manager_freq, motors[motIdx].emergency.bridge_off * 1000000);
    // Fixed step to slow down the motor
    motors[motIdx].motor_emergency.step = motors[motIdx].emergency.slope_time * motors[motIdx].manager_freq;
}

inline motor_safety_t get_motor_safety(short motIdx) {
    return motors[motIdx].safety;
}

void update_motor_safety(short motIdx, motor_safety_t safety) {
    // Store safety data
    memcpy(&motors[motIdx].safety, &safety, sizeof(motor_safety_t));
    // Init safety controller
    init_soft_timer(&motors[motIdx].motor_safety.stop, (motors[motIdx].manager_freq / 1000), safety.timeout * 1000000);
    // Initialization auto recovery system
    init_soft_timer(&motors[motIdx].motor_safety.restore, (motors[motIdx].manager_freq / 1000), safety.autorestore * 1000000);
    // Fixed step to slow down the motor in function of stop time
    motors[motIdx].motor_safety.step = (safety.timeout * motors[motIdx].manager_freq) / 1000;
}

inline motor_t get_motor_measures(short motIdx) {
    motors[motIdx].measure.position_delta = motors[motIdx].k_ang * motors[motIdx].PulsEnc;
    motors[motIdx].measure.position = motors[motIdx].k_ang * motors[motIdx].enc_angle;
    motors[motIdx].measure.current = motors[motIdx].parameter_motor.rotation * motors[motIdx].measure.current;
    // Torque in [m Nm]
    motors[motIdx].measure.effort = (motors[motIdx].measure.current * motors[motIdx].diagnostic.volt) / motors[motIdx].measure.velocity;
    motors[motIdx].PulsEnc = 0;
    return motors[motIdx].measure;
}

inline motor_t get_motor_control(short motIdx) {
    motors[motIdx].controlOut.current = motors[motIdx].controller[GET_CONTROLLER_NUM(CONTROL_CURRENT)].PIDstruct.controlOutput;
    return motors[motIdx].controlOut;
}

inline motor_diagnostic_t get_motor_diagnostic(short motIdx) {
    // get time execution
    motors[motIdx].diagnostic.time_control = get_time(motors[motIdx].motor_manager_event);
    // Evaluate the Watt required [mW]
    motors[motIdx].diagnostic.watt = (motors[motIdx].measure.current * motors[motIdx].diagnostic.volt) / 1000;
    return motors[motIdx].diagnostic;
}

inline motor_t get_motor_reference(short motIdx) {
    return motors[motIdx].reference;
}
             
inline void reset_motor_position_measure(short motIdx, motor_control_t value) {
    motors[motIdx].enc_angle = (int)((value / ((float)motors[motIdx].k_ang) ));
    motors[motIdx].measure.position = (float) value;  
}
             
void set_motor_reference(short motIdx, motor_state_t state, motor_control_t reference) {
    // If the controller is in safety state the new reference is skipped
    if(motors[motIdx].diagnostic.state != CONTROL_SAFETY) {
        // Check state
        if(state != motors[motIdx].state) {
            // Change motor control type
            set_motor_state(motIdx, state);
            // Update reference
            motors[motIdx].state = state;
        }
        // Setup reference
        motors[motIdx].external_reference = reference;
    }
    // Reset time emergency
    reset_timer(&motors[motIdx].motor_emergency.alive);
}
    
inline motor_state_t get_motor_state(short motIdx) {
    return motors[motIdx].diagnostic.state;
}

void set_motor_state(short motIdx, motor_state_t state) {
    bool enable = (state == CONTROL_DISABLE) ? false : true;
    // For all error controller the blinks are disabled
    int led_state = (state < CONTROL_DISABLE) ? state : state + 1;
    
    // Restore safety control when auto restore is disabled
    if(motors[motIdx].safety.autorestore == 0 && motors[motIdx].state == CONTROL_SAFETY && state != CONTROL_SAFETY) {
        restore_safety_control(motIdx);
    }
    
    /// Set enable or disable motors
    motors[motIdx].state = state;
    if(enable == (motors[motIdx].parameter_motor.bridge.enable == MOTOR_ENABLE_LOW)) {
        REGISTER_MASK_SET_HIGH(motors[motIdx].pin_enable->CS_PORT, motors[motIdx].pin_enable->CS_mask);
    } else {
        REGISTER_MASK_SET_LOW(motors[motIdx].pin_enable->CS_PORT, motors[motIdx].pin_enable->CS_mask);
    }
    
    // Store last velocity if run some error mode
    if (state < CONTROL_DISABLE) {
        motors[motIdx].last_reference = motors[motIdx].reference.velocity;
    }
    /// Disable PWM generator
    if (enable) {
        PTCONbits.PTEN = 1;
    } else {
        if ((motors[MOTOR_ZERO].state == CONTROL_DISABLE) && 
                (motors[MOTOR_ONE].state == CONTROL_DISABLE)) {
            PTCONbits.PTEN = 0;
        }
    }
#ifndef MOTION_CONTROL
    UpdateBlink(motIdx, led_state);
#elif MOTION_CONTROL
    UpdateBlink(LED1, led_state);
#endif
}

inline __attribute__((always_inline)) int castToDSP(motor_control_t value, motor_control_t constraint, volatile fractional *saturation) {
    // Check constraint
    motor_control_t int_const = constraint;
    if(labs(constraint) > INT16_MAX) {
        int_const = INT16_MAX;
    }
    // Check size
    if(value > int_const) {
        *saturation = int_const - value;
        return int_const;
    } else if(value < (-int_const-1)) {
        *saturation = (-int_const-1) - value;
        return (-int_const-1);
    } else {
        *saturation = 0;
        return value;
    }
}

inline int __attribute__((always_inline)) control_velocity(short motIdx, motor_control_t reference, volatile fractional saturation) {
#define CONTROLLER_VEL GET_CONTROLLER_NUM(CONTROL_VELOCITY)
    // Set reference
    motors[motIdx].controller[CONTROLLER_VEL].PIDstruct.controlReference = castToDSP(reference, motors[motIdx].constraint.velocity, &motors[motIdx].controller[CONTROLLER_VEL].saturation);
    motors[motIdx].reference.velocity = motors[motIdx].controller[CONTROLLER_VEL].PIDstruct.controlReference;
    // Set measure
    if(motors[motIdx].measure.velocity > INT16_MAX) {
        motors[motIdx].controller[CONTROLLER_VEL].PIDstruct.measuredOutput = INT16_MAX;
    } else if(motors[motIdx].measure.velocity < INT16_MIN) {
        motors[motIdx].controller[CONTROLLER_VEL].PIDstruct.measuredOutput = INT16_MIN;
    } else {
        motors[motIdx].controller[CONTROLLER_VEL].PIDstruct.measuredOutput = motors[motIdx].measure.velocity;
    }
    // Add anti wind up saturation from PWM
    // Add coefficient K_back calculation for anti wind up
    motors[motIdx].controller[CONTROLLER_VEL].PIDstruct.controlOutput += 
            motors[motIdx].controller[CONTROLLER_VEL].k_aw * saturation;
    // PID execution
    PID(&motors[motIdx].controller[CONTROLLER_VEL].PIDstruct);
    // Set Output
    motors[motIdx].controlOut.velocity = motors[motIdx].controller[CONTROLLER_VEL].PIDstruct.controlOutput;
    return motors[motIdx].controlOut.velocity;
#undef CONTROLLER_VEL
}

inline int __attribute__((always_inline)) control_current(short motIdx, motor_control_t reference, volatile fractional saturation) {
#define CONTROLLER_CURR GET_CONTROLLER_NUM(CONTROL_CURRENT)
    // Set reference
    motors[motIdx].controller[CONTROLLER_CURR].PIDstruct.controlReference = castToDSP(reference, motors[motIdx].constraint.current, &motors[motIdx].controller[CONTROLLER_CURR].saturation);
    motors[motIdx].reference.current = motors[motIdx].controller[CONTROLLER_CURR].PIDstruct.controlReference;
    // Set measure        
    if(motors[motIdx].measure.current > INT16_MAX) {
        motors[motIdx].controller[CONTROLLER_CURR].PIDstruct.measuredOutput = INT16_MAX;
    } else if(motors[motIdx].measure.current < INT16_MIN) {
        motors[motIdx].controller[CONTROLLER_CURR].PIDstruct.measuredOutput = INT16_MIN;
    } else {
        motors[motIdx].controller[CONTROLLER_CURR].PIDstruct.measuredOutput = motors[motIdx].measure.current;
    }
    // Add anti wind up saturation from PWM
    // Add coefficient K_back calculation for anti wind up
    motors[motIdx].controller[CONTROLLER_CURR].PIDstruct.controlOutput += 
            motors[motIdx].controller[CONTROLLER_CURR].k_aw * saturation;
    // PID execution
    PID(&motors[motIdx].controller[CONTROLLER_CURR].PIDstruct);
    // Get Output
    return motors[motIdx].controller[CONTROLLER_CURR].PIDstruct.controlOutput;
#undef CONTROLLER_CURR
}

inline void __attribute__((always_inline)) CurrentControl(short motIdx, int current, int voltage) {
#define CONTROLLER_CURR GET_CONTROLLER_NUM(CONTROL_CURRENT)
    motors[motIdx].measure.current = - motors[motIdx].current.gain * current + motors[motIdx].current.offset;
    motors[motIdx].diagnostic.volt = motors[motIdx].volt.gain * voltage + motors[motIdx].volt.offset;

    if(motors[motIdx].controller[CONTROLLER_CURR].enable) {
        // Set measure        
        if(motors[motIdx].measure.current > INT16_MAX) {
            motors[motIdx].controller[CONTROLLER_CURR].PIDstruct.measuredOutput = INT16_MAX;
        } else if(motors[motIdx].measure.current < INT16_MIN) {
            motors[motIdx].controller[CONTROLLER_CURR].PIDstruct.measuredOutput = INT16_MIN;
        } else {
            motors[motIdx].controller[CONTROLLER_CURR].PIDstruct.measuredOutput = motors[motIdx].measure.current;
        }
        // Add anti wind up saturation from PWM
        // Add coefficient K_back calculation for anti wind up
        motors[motIdx].controller[CONTROLLER_CURR].PIDstruct.controlOutput += 
                motors[motIdx].controller[CONTROLLER_CURR].k_aw * motors[motIdx].pwm_saturation;
        // PID execution
        PID(&motors[motIdx].controller[CONTROLLER_CURR].PIDstruct);
        // Set Output        
        motors[motIdx].pwm_saturation = Motor_PWM(motIdx, motors[motIdx].controller[CONTROLLER_CURR].PIDstruct.controlOutput);
    }
#undef CONTROLLER_CURR
}

void MotorTaskController(int argc, int *argv) {
    short motIdx = (short) argv[0];
    
    // ================ SAFETY CHECK ===========================
    /// Check for emergency mode or in safety mode
    if (motors[motIdx].state > CONTROL_DISABLE) {
        // Check safety condition
        if (labs(motors[motIdx].measure.current) > motors[motIdx].safety.critical_zone) {
            // Run timer if current is too high
            if (run_timer(&motors[motIdx].motor_safety.stop)) {
                // Store old state
                motors[motIdx].motor_safety.old_state = motors[motIdx].state;
                // Change motor state
                set_motor_state(motIdx, CONTROL_SAFETY);
            }
        } else {
            // Reset the counter if the value is low
            reset_timer(&motors[motIdx].motor_safety.stop);
        }
    }
    // Check emergency 
    if(motors[motIdx].state != CONTROL_EMERGENCY && motors[motIdx].state != CONTROL_DISABLE) {
        // check to run emergency mode
        if(run_timer(&motors[motIdx].motor_emergency.alive)) {
            /// Set Motor in emergency mode
            set_motor_state(motIdx, CONTROL_EMERGENCY);
            // Reset stop timer
            reset_timer(&motors[motIdx].motor_emergency.stop);
        }
    }
    // ================ RUN CONTROLLERS ========================
    /// Add new task controller
    if(motors[motIdx].state != motors[motIdx].diagnostic.state) {
        if(motors[motIdx].state == CONTROL_EMERGENCY) {
            if(motors[motIdx].diagnostic.state == CONTROL_SAFETY) {
                // Stop safety task
                task_set(motors[motIdx].motor_safety.task_safety, STOP);
            }
            task_set(motors[motIdx].task_emergency, RUN);
        } else {
            task_set(motors[motIdx].task_emergency, STOP);
        }
        if(motors[motIdx].state == CONTROL_SAFETY) {
            task_set(motors[motIdx].motor_safety.task_safety, RUN);
        }
        /// Save new state controller
        motors[motIdx].diagnostic.state = motors[motIdx].state;
    }
    // ================ READ MEASURES ==========================
    
    bool velocity_control = run_timer(&motors[motIdx].controller[GET_CONTROLLER_NUM(CONTROL_VELOCITY)].timer);
    
    // Check if is the time to run the controller
    if (velocity_control) {
        //Measure velocity in milli rad/s
        motors[motIdx].measure.velocity = (motor_control_t) measureVelocity(motIdx);
    }
    
    // ================ RUN CONTROLLERS ==========================
    
    // If some controller is selected
    if (motors[motIdx].state != CONTROL_DISABLE) {
        // ========== CONTROL DIRECT =============
        if(motors[motIdx].state == CONTROL_DIRECT) {
            // Send to motor the value of control
            Motor_PWM(motIdx, motors[motIdx].external_reference);
            return;
        }
        // ========= CONTROL VELOCITY ============
#define ENABLE_VELOCITY_CONTROL
        // Check if the velocity control is enabled
        if (motors[motIdx].controller[GET_CONTROLLER_NUM(CONTROL_VELOCITY)].enable) {
            // Check if is the time to run the controller
            if (velocity_control) {
                // Run PID control
                volatile fractional saturation = 0;
                if(motors[motIdx].controller[GET_CONTROLLER_NUM(CONTROL_CURRENT)].enable == true) {
                    saturation = motors[motIdx].controller[GET_CONTROLLER_NUM(CONTROL_CURRENT)].saturation;
                } else {
                    saturation = motors[motIdx].pwm_saturation;
                }
#ifdef ENABLE_VELOCITY_CONTROL
                motors[motIdx].control_output = control_velocity(motIdx, motors[motIdx].external_reference, saturation);
#else
                control_velocity(motIdx, motors[motIdx].external_reference, saturation);
                motors[motIdx].control_output = 0;
#endif
            }
        } else {
            // Set the reference for the other current control the same of the reference
            motors[motIdx].control_output = motors[motIdx].external_reference;
        }
        // =======================================

#ifndef CURRENT_CONTROL_IN_ADC_LOOP
        // ========= CONTROL CURRENT =============
        // Check if the current control is enabled
        if (motors[motIdx].controller[GET_CONTROLLER_NUM(CONTROL_CURRENT)].enable) {
            // Check if is the time to run the controller
            if (run_controller(&motors[motIdx].controller[GET_CONTROLLER_NUM(CONTROL_CURRENT)])) {
                // The current is evaluated with sign of motor rotation
                // Update current and volt values;
                motors[motIdx].measure.current = -motors[motIdx].current.gain * gpio_get_analog(0, motors[motIdx].pin_current)
                        + motors[motIdx].current.offset;
                motors[motIdx].diagnostic.volt = motors[motIdx].volt.gain * gpio_get_analog(0, motors[motIdx].pin_voltage)
                        + motors[motIdx].volt.offset;
                // Run PID control
                motors[motIdx].control_output = control_current(motIdx, motors[motIdx].external_reference, motors[motIdx].pwm_saturation);

                // Send to motor the value of control
                Motor_PWM(motIdx, motors[motIdx].parameter_motor.rotation * motors[motIdx].control_output);
            }
        }
        // =======================================
#else
        // If disabled Send the PWM after this line
        if (motors[motIdx].controller[GET_CONTROLLER_NUM(CONTROL_CURRENT)].enable) {
            // Set current reference
            motors[motIdx].controller[GET_CONTROLLER_NUM(CONTROL_CURRENT)].PIDstruct.controlReference = castToDSP(motors[motIdx].control_output, 
                    motors[motIdx].constraint.current,
                    &motors[motIdx].controller[GET_CONTROLLER_NUM(CONTROL_CURRENT)].saturation);
            motors[motIdx].reference.current = motors[motIdx].controller[GET_CONTROLLER_NUM(CONTROL_CURRENT)].PIDstruct.controlReference;
        } else {
            // Send to motor the value of control
            motors[motIdx].pwm_saturation = Motor_PWM(motIdx, motors[motIdx].control_output);
        }
#endif
    }
}

int32_t measureVelocity(short motIdx) {
    volatile ICdata temp;
    int QEICNTtmp = 0;
    int32_t vel_mean = 0;
    //Evaluate position
    switch (motIdx) {
        case MOTOR_ZERO:
            QEICNTtmp = (int) POS1CNT;
            POS1CNT = 0;
            break;
        case MOTOR_ONE:
            QEICNTtmp = (int) POS2CNT;
            POS2CNT = 0;
            break;
    }
    motors[motIdx].PulsEnc += QEICNTtmp;
    motors[motIdx].enc_angle += QEICNTtmp;
    //Measure velocity from QEI
    int32_t vel_qei =  QEICNTtmp * motors[motIdx].k_vel_qei;

    // Store timePeriod
    temp.timePeriod = motors[motIdx].ICinfo->timePeriod;
    motors[motIdx].ICinfo->timePeriod = 0;
    // Store value
    temp.SIG_VEL = motors[motIdx].ICinfo->SIG_VEL;
    motors[motIdx].ICinfo->SIG_VEL = 0;
    if (temp.SIG_VEL) {

        temp.k_mul = motors[motIdx].ICinfo->k_mul;
        // Evaluate velocity
        int32_t temp_f = temp.k_mul * motors[motIdx].k_vel_ic;
        temp_f = temp_f / temp.timePeriod;
        int32_t vel_ic = temp.SIG_VEL * temp_f;

        if (labs(vel_qei - vel_ic) < 2000) {
            // Mean velocity between IC and QEI estimation
            vel_mean = (vel_ic + vel_qei) / 2;
        } else {
            vel_mean = vel_qei;
        }
    } else {
        vel_mean = vel_qei;
    }
    //Select Prescaler
    motors[motIdx].prescaler_callback(motIdx);
    // Evaluate angle position
    if (labs(motors[motIdx].enc_angle) > motors[motIdx].angle_limit) {
        motors[motIdx].enc_angle = 0;
    }
    return update_statistic(&motors[motIdx].mean_vel, vel_mean);
}

#define SATURATION
inline int Motor_PWM(short motIdx, int duty_cycle) {
    // Store the real value send to the PWM
    motors[motIdx].reference.pwm = duty_cycle;
#ifdef SATURATION
    int error = 0;
    if(duty_cycle > motors[motIdx].pwm_limit) {
        error = motors[motIdx].pwm_limit - duty_cycle;
        duty_cycle = motors[motIdx].pwm_limit;
    } else if(duty_cycle < (-motors[motIdx].pwm_limit-1)) {
        error = (-motors[motIdx].pwm_limit-1) - duty_cycle;
        duty_cycle = (-motors[motIdx].pwm_limit-1);
    }
#else
    // Save PWM value with attenuation => K = 1 / 16
    duty_cycle = duty_cycle >> 4;
#endif
    // Real PWM send
    motors[motIdx].measure.pwm = duty_cycle;
    // PWM output
    if(motors[motIdx].state != CONTROL_DISABLE)
        SetDCMCPWM1(motIdx + 1, DEFAULT_PWM_OFFSET + ((motor_control_t) motors[motIdx].parameter_motor.rotation) * duty_cycle, 0);
    else
        SetDCMCPWM1(motIdx + 1, DEFAULT_PWM_OFFSET, 0);
#ifdef SATURATION
    return error;
#else
    return 0;
#endif
}

void Emergency(int argc, int *argv) {
    short motIdx = (short) argv[0];
    if (motors[motIdx].external_reference != 0) {
        motors[motIdx].external_reference -= motors[motIdx].last_reference / motors[motIdx].motor_emergency.step;
        if (SGN(motors[motIdx].external_reference) * motors[motIdx].last_reference < 0) {
            motors[motIdx].external_reference = 0;
        }
    } else if (motors[motIdx].external_reference == 0) {
        if(run_timer(&motors[motIdx].motor_emergency.stop)) {
            set_motor_state(motIdx, CONTROL_DISABLE);
        }
    }
}

void restore_safety_control(short motIdx) {
    // Reset stop time
    reset_timer(&motors[motIdx].motor_safety.stop);
    // Stop safety controller
    task_set(motors[motIdx].motor_safety.task_safety, STOP);
}

void Safety(int argc, int *argv) {
    short motIdx = (short) argv[0];
    // Reduction external reference
    if(labs(motors[motIdx].measure.current) > motors[motIdx].safety.critical_zone) {
        if (motors[motIdx].external_reference != 0) {
            motors[motIdx].external_reference -= motors[motIdx].last_reference / motors[motIdx].motor_safety.step;
            if (SGN(motors[motIdx].external_reference) * motors[motIdx].last_reference < 0) {
                motors[motIdx].external_reference = 0;
            }
        }
        // Reset recovery time
        reset_timer(&motors[motIdx].motor_safety.restore);
    } else {
        // Check if auto restore is disabled
        if(motors[motIdx].safety.autorestore != 0) {
            // Run auto recovery timer
            if(run_timer(&motors[motIdx].motor_safety.restore)) {
                // Restore control
                restore_safety_control(motIdx);
                // Restore old controller
                set_motor_state(motIdx, motors[motIdx].motor_safety.old_state);
            }
        }
    }
}