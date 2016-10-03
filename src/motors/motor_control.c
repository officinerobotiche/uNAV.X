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
#define DEFAULT_VOLT_BRIDGE 3
#define DEFAULT_CPR 300
#define DEFAULT_RATIO 30
#define DEFAULT_ENC_POSITION MOTOR_ENC_AFTER_GEAR
#define DEFAULT_ENC_CHANNELS MOTOR_ENC_CHANNEL_TWO
#define DEFAULT_ENC_Z_INDEX MOTOR_ENC_Z_INDEX_NO
#define DEFAULT_VERSUS_ROTATION MOTOR_ROTATION_COUNTERCLOCKWISE
#define DEFAULT_MOTOR_ENABLE MOTOR_ENABLE_LOW

#define DEFAULT_FREQ_MOTOR_MANAGER 10000             // Task Manager 10Khz
#define DEFAULT_FREQ_MOTOR_CONTROL_EMERGENCY 1000   // In Herts

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

typedef struct _analog {
    int32_t gain;
    int32_t offset;
} analog_t;

typedef struct _timer {
    uint32_t time;
    uint32_t counter;
} timer_t;

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
    // time to launch PID controller
    unsigned int time;
    // internal time counter
    unsigned int counter;
    // enable
    volatile bool enable;
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
        timer_t alive;
        timer_t stop;
        uint16_t step;
        motor_control_t last_reference;
    } motor_emergency;
    motor_emergency_t emergency;
    bool save_velocity;
    //gain motor
    int32_t k_vel_ic, k_vel_qei;
    float k_ang;
    // Velocity mean
    statistic_buffer mean_vel;
    //Internal value volt and current
    analog_t volt, current;
    //PID
    volatile motor_control_t external_reference;
    volatile motor_control_t control_output;
    //pid_controller_t controller;
    pid_controller_t controller[NUM_CONTROLLERS];
    //Common
    motor_diagnostic_t diagnostic;
    motor_parameter_t parameter_motor;
    bool currentControlInside;
    motor_t controlOutput;
    motor_t constraint;
    motor_t controlOut;
    motor_t reference;
    motor_t measure;
} motor_firmware_t;
motor_firmware_t motors[NUM_MOTORS];

/*****************************************************************************/
/* User Functions                                                            */
/*****************************************************************************/

void reset_motor_data(motor_t* motor) {
    motor->position_delta = 0;
    motor->position = 0;
    motor->velocity = 0;
    motor->current = 0;
    motor->pwm = 0;
    motor->state = CONTROL_DISABLE;
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
        // Initialize PID counter time
        motors[motIdx].controller[i].counter = 0;
        // Initialize PID time
        motors[motIdx].controller[i].time = 0;
        // Enable
        motors[motIdx].controller[i].enable = false;
    }
}

hTask_t init_motor(const short motIdx, gpio_t* enable_, ICdata* ICinfo_, event_prescaler_t prescaler_event, int current_, int voltage_) {
    reset_motor_data(&motors[motIdx].measure);
    reset_motor_data(&motors[motIdx].reference);
    reset_motor_data(&motors[motIdx].controlOut);
    
    motors[motIdx].control_output = 0;
    motors[motIdx].external_reference = 0;
    //Initialize controllers
    initialize_controllers(motIdx);
/*
    // Set if current control run in ADC callback
    motors[motIdx].currentControlInside = true;
*/
    motors[motIdx].prescaler_callback = prescaler_event;
    // Setup frequency task manager
    motors[motIdx].manager_freq = DEFAULT_FREQ_MOTOR_MANAGER;
    //Setup diagnostic
    motors[motIdx].diagnostic.time_control = 0;
    motors[motIdx].diagnostic.watt = 0;
    motors[motIdx].diagnostic.temperature = 0;
    motors[motIdx].diagnostic.time_control = 0;
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
    // Return Task manager
    return motors[motIdx].task_manager;
}

void set_currentControlInside(short motIdx, bool value) {
    motors[motIdx].currentControlInside = value;
}

motor_parameter_t init_motor_parameters() {
    motor_parameter_t parameter;
    parameter.ratio = (float) DEFAULT_RATIO; //Gain to convert QEI value to rotation movement
    parameter.rotation = DEFAULT_VERSUS_ROTATION;
    parameter.bridge.enable = DEFAULT_MOTOR_ENABLE;
    parameter.bridge.pwm_dead_zone = 0;
    parameter.bridge.pwm_frequency = 0;
    parameter.bridge.volt_offset = DEFAULT_VOLT_BRIDGE;
    parameter.bridge.volt_gain = 1;
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
    motors[motIdx].k_vel_qei = (motors[motIdx].k_ang * 1000.0f * motors[motIdx].manager_freq);
    
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
    motors[motIdx].volt.gain = 1000.0 * motors[motIdx].parameter_motor.bridge.volt_gain;
    motors[motIdx].volt.offset = 1000.0 * motors[motIdx].parameter_motor.bridge.volt_offset;
    // Setup state with new bridge configuration
    set_motor_state(motIdx, motors[motIdx].measure.state);
}

motor_t init_motor_constraints() {
    motor_t constraint;
    constraint.state = STATE_CONTROL_DISABLE;
    constraint.position = MOTOR_CONTROL_MAX;
    constraint.current = MOTOR_CONTROL_MAX;
    constraint.pwm = MOTOR_CONTROL_MAX;
    constraint.velocity = MOTOR_CONTROL_MAX;
    return constraint;
}

inline motor_t get_motor_constraints(short motIdx) {
    return motors[motIdx].constraint;
}

void update_motor_constraints(short motIdx, motor_t constraints) {
    //Update parameter constraints
    memcpy(&motors[motIdx].constraint, &constraints, sizeof(motor_t));
}

inline motor_pid_t get_motor_pid(short motIdx, motor_state_t state) {
    int num_control = GET_CONTROLLER_NUM(state);
    return motors[motIdx].controller[num_control].pid;
}

bool update_motor_pid(short motIdx, motor_state_t state, motor_pid_t pid) {
    // Check gains value
    // Check1 = | Kp + ki + kd | < 1 = INT16_MAX
    // Check2 = | -(Kp + 2*Kd) | < 1 = INT16_MAX
    // Check3 =      | Kd |      < 1 = INT16_MAX
    long check1 = labs(1000.0 * (pid.kp + pid.ki + pid.kd));
    long check2 = labs(1000.0 * (pid.kp + 2 * pid.kd));
    
    if(check1 < INT16_MAX && check2 < INT16_MAX && labs(pid.kd) < INT16_MAX) {
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
        // Write new time
        motors[motIdx].controller[num_control].time = motors[motIdx].manager_freq / pid.frequency;
        // reset counter
        motors[motIdx].controller[num_control].counter = 0;
        
        motors[motIdx].controller[num_control].enable = pid.enable;
        return true;
    } else
        return false;
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
    // Store emergency data
    memcpy(&motors[motIdx].emergency, &emergency_data, sizeof(motor_emergency_t));
    // Reset counter alive reference message
    motors[motIdx].motor_emergency.alive.time = emergency_data.timeout * (motors[motIdx].manager_freq / 1000);
    motors[motIdx].motor_emergency.alive.counter = 0;
    // Reset counter Emergency stop
    motors[motIdx].motor_emergency.stop.time = motors[motIdx].emergency.bridge_off * motors[motIdx].manager_freq;
    motors[motIdx].motor_emergency.stop.counter = 0;
    // Fix step to slow down the motor
    motors[motIdx].motor_emergency.step = motors[motIdx].emergency.slope_time * motors[motIdx].manager_freq;
}

inline motor_t get_motor_measures(short motIdx) {
    motors[motIdx].measure.position_delta = motors[motIdx].k_ang * motors[motIdx].PulsEnc;
    motors[motIdx].measure.position = motors[motIdx].k_ang * motors[motIdx].enc_angle; 
    motors[motIdx].PulsEnc = 0;
    return motors[motIdx].measure;
}

inline motor_t get_motor_control(short motIdx) {
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
    // Check state
    if(state != motors[motIdx].measure.state) {
        // Change motor control type
        set_motor_state(motIdx, state);
        // Update reference
        motors[motIdx].reference.state = state;
    }
    // Setup reference
    motors[motIdx].external_reference = reference;
//    switch (state) {
//        case CONTROL_POSITION:
//            motors[motIdx].reference.position = reference;
//            break;
//        case CONTROL_VELOCITY:
//            motors[motIdx].reference.velocity = reference;
//            break;
//        case CONTROL_CURRENT:
//            motors[motIdx].reference.current = reference;
//            break;
//    }
    // Reset time emergency
    motors[motIdx].motor_emergency.alive.counter = 0; 
}
    
inline motor_state_t get_motor_state(short motIdx) {
    return motors[motIdx].measure.state;
}

void set_motor_state(short motIdx, motor_state_t state) {
    volatile bool enable = (state != CONTROL_DISABLE) ? true : false;
    int led_state = (state != CONTROL_EMERGENCY) ? state + 1 : state;
    
    /// Set enable or disable motors
    motors[motIdx].reference.state = state;
    if(enable == (motors[motIdx].parameter_motor.bridge.enable == MOTOR_ENABLE_LOW)) {
        REGISTER_MASK_SET_HIGH(motors[motIdx].pin_enable->CS_PORT, motors[motIdx].pin_enable->CS_mask);
    } else {
        REGISTER_MASK_SET_LOW(motors[motIdx].pin_enable->CS_PORT, motors[motIdx].pin_enable->CS_mask);
    }
    
    if (state == CONTROL_EMERGENCY) {
        motors[motIdx].motor_emergency.last_reference = motors[motIdx].reference.velocity;
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

inline int castToDSP(motor_control_t value, motor_control_t constraint) {
    // Check constraint
    if (abs(value) > constraint) {
        value = SGN(value) * constraint;
    }
    // Check size
    if(value > INT16_MAX)
        return INT16_MAX;
    else if(value < INT16_MIN)
        return INT16_MIN;
    else
        return value;
}

bool run_controller(pid_controller_t *controller) {
    // Check if is the time to run
    if(controller->counter >= controller->time) {
        return true;
    } else {
        // Increase the counter
        controller->counter++;
    }
    return false;
}

int control_velocity(short motIdx, motor_control_t reference) {
    int num_control = GET_CONTROLLER_NUM(CONTROL_VELOCITY);
    // Set reference
    motors[motIdx].controller[num_control].PIDstruct.controlReference = castToDSP(reference, motors[motIdx].constraint.velocity);
    motors[motIdx].reference.velocity = motors[motIdx].controller[num_control].PIDstruct.controlReference;
    // Set measure
    motors[motIdx].controller[num_control].PIDstruct.measuredOutput = castToDSP(motors[motIdx].measure.velocity, INT16_MAX);
    // PID execution
    PID(&motors[motIdx].controller[num_control].PIDstruct);
    // Set Output
    motors[motIdx].controlOut.velocity = motors[motIdx].parameter_motor.rotation * motors[motIdx].controller[num_control].PIDstruct.controlOutput;
    return motors[motIdx].controlOut.velocity;
}

int control_current(short motIdx, motor_control_t reference) {
    int num_control = GET_CONTROLLER_NUM(CONTROL_CURRENT);
    // Set reference
    motors[motIdx].controller[num_control].PIDstruct.controlReference = castToDSP(reference, motors[motIdx].constraint.current);
    motors[motIdx].reference.current = motors[motIdx].controller[num_control].PIDstruct.controlReference;
    // Set measure
    motors[motIdx].controller[num_control].PIDstruct.measuredOutput = - castToDSP(motors[motIdx].measure.current, INT16_MAX);
    // PID execution
    PID(&motors[motIdx].controller[num_control].PIDstruct);
    // Set Output
    if(motors[motIdx].controller[num_control].PIDstruct.controlOutput == -32768) {
        motors[motIdx].controlOut.current = 0;
    } else {
        motors[motIdx].controlOut.current = motors[motIdx].controller[num_control].PIDstruct.controlOutput;
    }
    return motors[motIdx].controlOut.current;
}

inline void CurrentControl(short motIdx, int current, int voltage) {
    motors[motIdx].measure.current = motors[motIdx].current.gain * current - motors[motIdx].current.offset;
    motors[motIdx].diagnostic.volt = motors[motIdx].volt.gain * voltage + motors[motIdx].volt.offset;
    
    if(motors[motIdx].controller[2].enable) {
        int control_output = control_current(motIdx, motors[motIdx].control_output);

        // Send to motor the value of control
        Motor_PWM(motIdx, control_output);
    }
}

void MotorTaskController(int argc, int *argv) {
    short motIdx = (short) argv[0];
    /// Add new task controller
//    if(motors[motIdx].reference.state != motors[motIdx].measure.state) {
//        if(motors[motIdx].reference.state == CONTROL_EMERGENCY) {
//            task_set(motors[motIdx].task_emergency, RUN);
//        } else {
//            task_set(motors[motIdx].task_emergency, STOP);
//        }
//        /// Save new state controller
//        motors[motIdx].measure.state = motors[motIdx].reference.state;
//    }
//    /// Check for emergency mode
//    if(motors[motIdx].reference.state > CONTROL_DISABLE) {
//        if ((motors[motIdx].motor_emergency.alive.counter + 1) >= motors[motIdx].motor_emergency.alive.time) {
//            /// Set Motor in emergency mode
//            set_motor_state(motIdx, CONTROL_EMERGENCY);
//            motors[motIdx].motor_emergency.stop.counter = 0;
//            motors[motIdx].motor_emergency.alive.counter = 0;
//        } else
//            motors[motIdx].motor_emergency.alive.counter++;
//    }
    //-------------- BUILD MEASURES --------------------------------------------

    //Measure velocity in milli rad/s
//    motors[motIdx].measure.velocity = (motor_control_t) measureVelocity(motIdx);
    // Update current and volt values;
    // The current is evaluated with sign of motor rotation
#ifndef INTERNAL_CONTROL
    motors[motIdx].measure.current = motors[motIdx].current.gain * gpio_get_analog(0, motors[motIdx].pin_current)
                            - motors[motIdx].current.offset;
    motors[motIdx].diagnostic.volt = motors[motIdx].volt.gain * gpio_get_analog(0, motors[motIdx].pin_voltage) 
                            + motors[motIdx].volt.offset;
#endif
    //-------------- PID CONTROL -----------------------------------------------
#define DEBUG
    // ======= TEST CONTROL VELOCITY =========
#ifdef DEBUG_WITH_VELOCITY
    int num_control = GET_CONTROLLER_NUM(CONTROL_VELOCITY);
    if(run_controller(&motors[motIdx].controller[num_control])) {
        //Measure velocity in milli rad/s
        motors[motIdx].measure.velocity = (motor_control_t) measureVelocity(motIdx);
        // Set reference
        motors[motIdx].controller[num_control].PIDstruct.controlReference = castToDSP(motors[motIdx].external_reference, motors[motIdx].constraint.velocity);
        motors[motIdx].reference.velocity = motors[motIdx].controller[num_control].PIDstruct.controlReference;
        // Set measure
        motors[motIdx].controller[num_control].PIDstruct.measuredOutput = castToDSP(motors[motIdx].measure.velocity, INT16_MAX);
        // PID execution
        PID(&motors[motIdx].controller[num_control].PIDstruct);
        
        motors[motIdx].control_output = motors[motIdx].controller[num_control].PIDstruct.controlOutput;
    }
#else
     motors[motIdx].control_output = 0;
#endif
    // =======================================

    // ======= TEST CONTROL CURRENT ==========
#ifdef DEBUG
    if(motIdx == MOTOR_ZERO) {
        motors[motIdx].control_output = control_current(motIdx, motors[motIdx].control_output);
    } else {
        motors[motIdx].control_output = 0;
    }
#else
    motors[motIdx].control_output = control_current(motIdx, motors[motIdx].external_reference);
#endif
    // =======================================


#ifndef INTERNAL_CONTROL
    // Send to motor the value of control
    Motor_PWM(motIdx, motors[motIdx].control_output);
#endif
}

int32_t measureVelocity(short motIdx) {
    volatile ICdata temp;
    int QEICNTtmp;
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
inline void Motor_PWM(short motIdx, int pwm_control) {

#ifdef SATURATION
    if(pwm_control > 2048) pwm_control = 2048;
    else if(pwm_control < -2048) pwm_control = -2048;
#else
    // Save PWM value with attenuation => K = 1 / 16
    pwm_control = pwm_control >> 4;
    motors[motIdx].measure.pwm = pwm_control * motors[motIdx].parameter_motor.rotation;
#endif
    // PWM output
    SetDCMCPWM1(motIdx + 1, pwm_control + DEFAULT_PWM_OFFSET, 0);
}

void Emergency(int argc, int *argv) {
    short motIdx = (short) argv[0];
    if (motors[motIdx].reference.velocity != 0) {
        motors[motIdx].reference.velocity -= motors[motIdx].motor_emergency.last_reference / motors[motIdx].motor_emergency.step;
        if (SGN(motors[motIdx].reference.velocity) * motors[motIdx].motor_emergency.last_reference < 0) {
            motors[motIdx].reference.velocity = 0;
        }
    } else if (motors[motIdx].reference.velocity == 0) {
        if ((motors[motIdx].motor_emergency.stop.counter + 1) >= motors[motIdx].motor_emergency.stop.time) {
            set_motor_state(motIdx, CONTROL_DISABLE);
            motors[motIdx].motor_emergency.stop.counter = 0;
        } else
            motors[motIdx].motor_emergency.stop.counter++;
    }
}
