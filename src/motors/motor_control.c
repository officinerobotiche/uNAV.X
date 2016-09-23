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
#define GAIN_ADC (ADC_AVSS/INT16_MAX)
/**
 * Default value for motor parameters
 */
#define DEFAULT_VOLT_BRIDGE 3
#define DEFAULT_CPR 300
#define DEFAULT_RATIO 30
#define DEFAULT_CASCADE_CONTROL 0
#define DEFAULT_ENC_POSITION MOTOR_ENC_AFTER_GEAR
#define DEFAULT_ENC_CHANNELS MOTOR_ENC_CHANNEL_TWO
#define DEFAULT_ENC_Z_INDEX MOTOR_ENC_Z_INDEX_NO
#define DEFAULT_VERSUS_ROTATION MOTOR_ROTATION_COUNTERCLOCKWISE
#define DEFAULT_MOTOR_ENABLE MOTOR_ENABLE_LOW
/**
 * Default value for PID
 */
#define DEFAULT_KP 0.6
#define DEFAULT_KI 0.15
#define DEFAULT_KD 0.2

#define DEFAULT_FREQ_MOTOR_CONTROL_VELOCITY 1000    // In Herts

#define DEFAULT_FREQ_MOTOR_MANAGER 1000            // Task Manager 10Khz
#define DEFAULT_FREQ_MOTOR_CONTROL_EMERGENCY 1000   // In Herts

/*****************************************************************************/
/* Global Variable Declaration                                               */
/*****************************************************************************/

#define MOTOR "MOTOR"
static string_data_t _MODULE_MOTOR = {MOTOR, sizeof(MOTOR)};

/**
 * xc16 PID source in: folder_install_microchip_software/xc16/1.2x/src/libdsp.zip
 * on zip file: asm/pid.s
 */

#define NUM_CONTROLLERS 3
// Get the number in controller array from enum_state_t
#define GET_CONTROLLER_NUM(X) ((X) - 1)

fractional abcCoefficient1[NUM_CONTROLLERS][3] __attribute__((section(".xbss, bss, xmemory")));
fractional controlHistory1[NUM_CONTROLLERS][3] __attribute__((section(".ybss, bss, ymemory")));
fractional abcCoefficient2[NUM_CONTROLLERS][3] __attribute__((section(".xbss, bss, xmemory")));
fractional controlHistory2[NUM_CONTROLLERS][3] __attribute__((section(".ybss, bss, ymemory")));

/** */

#define DEFAULT_PWM_OFFSET 2048

typedef struct _analog_convert {
    float k;
    int offset;
    int value;
} analog_convert_t;

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
    unsigned int counter_alive;
    unsigned int counter_stop;
    /// Task register
    hTask_t task_manager;
    // Task manager for emergency control
    hTask_t task_emergency;
    // Last reference saved before emergency mode
    motor_control_t last_reference;
    /// Motor position
    uint32_t angle_limit;
    volatile int PulsEnc;
    volatile int32_t enc_angle;
    int rotation; //Check if required in future
    //Emergency
    float emergency_step;
    float emergency_stop;
    bool save_velocity;
    //gain motor
    int32_t k_vel_ic, k_vel_qei;
    float k_ang;
    // Velocity mean
    statistic_buffer mean_vel;
    //Internal value volt and current
    analog_convert_t volt, current;
    //Common
    motor_diagnostic_t diagnostic;
    motor_emergency_t emergency;
    motor_parameter_t parameter_motor;
    
    pid_controller_t controller[NUM_CONTROLLERS];
    motor_t controlOutput;
    motor_t constraint;
    motor_t reference;
    motor_t measure;
    event_prescaler_t prescaler_callback;
} motor_firmware_t;
motor_firmware_t motors[NUM_MOTORS];

/*****************************************************************************/
/* User Functions                                                            */
/*****************************************************************************/

void reset_motor_data(motor_t* motor) {
    motor->position_delta = 0;
    motor->position = 0;
    motor->velocity = 0;
    motor->torque = 0;
    motor->pwm = 0;
    motor->state = CONTROL_DISABLE;
}

void init_controllers(short motIdx) {
    int i;
    for(i = 0; i < NUM_CONTROLLERS; i++) {
        switch (motIdx) {
            case MOTOR_ZERO:
                //Initialize the PID data structure: PIDstruct
                //Set up pointer to derived coefficients
                motors[motIdx].controller[i].PIDstruct.abcCoefficients = &abcCoefficient1[i][0];
                //Set up pointer to controller history samples
                motors[motIdx].controller[i].PIDstruct.controlHistory = &controlHistory1[i][0];
                break;
            case MOTOR_ONE:
                //Initialize the PID data structure: PIDstruct
                //Set up pointer to derived coefficients
                motors[motIdx].controller[i].PIDstruct.abcCoefficients = &abcCoefficient2[i][0];
                //Set up pointer to controller history samples
                motors[motIdx].controller[i].PIDstruct.controlHistory = &controlHistory2[i][0];
                break;
        }
        // Clear the controller history and the controller output
        PIDInit(&motors[motIdx].controller[i].PIDstruct);
        // reset timers
        motors[motIdx].controller[i].counter = 0;
        motors[motIdx].controller[i].time = 0;
    }
}

hTask_t init_motor(const short motIdx, gpio_t* enable_, ICdata* ICinfo_, event_prescaler_t prescaler_event, int current_, int voltage_) {
    reset_motor_data(&motors[motIdx].measure);
    reset_motor_data(&motors[motIdx].reference);
    init_controllers(motIdx);
    motors[motIdx].prescaler_callback = prescaler_event;
    //Setup diagnostic
    motors[motIdx].diagnostic.current = 0;
    motors[motIdx].diagnostic.temperature = 0;
    // Register input capture
    motors[motIdx].ICinfo = ICinfo_;
    // Initialization direction of rotation
    motors[motIdx].rotation = 1;
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
    hEvent_t motor_manager_task = register_event_p(motor_manager_module, &MotorTaskController, EVENT_PRIORITY_MEDIUM);
    motors[motIdx].task_manager = task_load_data(motor_manager_task, DEFAULT_FREQ_MOTOR_MANAGER, 1, (char) motIdx);
    /// Load controller EMERGENCY - Working at 1KHz
    hModule_t emegency_module = register_module(&_MODULE_MOTOR);
    hEvent_t emergency_event = register_event_p(emegency_module, &Emergency, EVENT_PRIORITY_HIGH);
    motors[motIdx].task_emergency = task_load_data(emergency_event, DEFAULT_FREQ_MOTOR_CONTROL_EMERGENCY, 1, (char) motIdx);
    // Return Task manager
    return motors[motIdx].task_manager;
}

motor_parameter_t init_motor_parameters() {
    motor_parameter_t parameter;
    parameter.ratio = (float) DEFAULT_RATIO; //Gain to convert QEI value to rotation movement
    parameter.rotation = DEFAULT_VERSUS_ROTATION;
    parameter.cascade_control = DEFAULT_CASCADE_CONTROL;
    parameter.bridge.enable = DEFAULT_MOTOR_ENABLE;
    parameter.bridge.pwm_dead_zone = 0;
    parameter.bridge.pwm_frequency = 0;
    parameter.bridge.volt_offset = DEFAULT_VOLT_BRIDGE;
    parameter.bridge.volt_gain = 1;
    parameter.bridge.current_offset = 0;
    parameter.bridge.current_gain = 1;
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
    motors[motIdx].parameter_motor = parameters;
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
    motors[motIdx].k_vel_qei = (motors[motIdx].k_ang * 1000000.0f);
    
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
    motors[motIdx].current.k = (-motors[motIdx].parameter_motor.rotation) * GAIN_KILO * ((float) GAIN_ADC) / motors[motIdx].parameter_motor.bridge.current_gain;
    motors[motIdx].current.offset = ((int)(motors[motIdx].parameter_motor.bridge.current_offset / ((float) GAIN_ADC)));
    // Convert gain volt in [mV]
    if(motors[motIdx].parameter_motor.bridge.volt_gain == 0) {
        motors[motIdx].volt.offset = (int)(GAIN_KILO*motors[motIdx].parameter_motor.bridge.volt_offset);
        motors[motIdx].volt.k = 1;
    } else {
        motors[motIdx].volt.k = motors[motIdx].parameter_motor.bridge.volt_gain * GAIN_ADC;
        motors[motIdx].volt.offset = (int)(motors[motIdx].parameter_motor.bridge.volt_offset / motors[motIdx].volt.k);
        motors[motIdx].volt.k *= GAIN_KILO;
    }
}

motor_t init_motor_constraints() {
    motor_t constraint;
    constraint.state = 0;
    constraint.position = -1;
    constraint.torque = -1;
    constraint.pwm = -1;
    constraint.velocity = 25000;
    return constraint;
}

inline motor_t get_motor_constraints(short motIdx) {
    return motors[motIdx].constraint;
}

void update_motor_constraints(short motIdx, motor_t constraints) {
    //Update parameter constraints
    motors[motIdx].constraint = constraints;
}

motor_pid_t init_motor_pid() {
    motor_pid_t pid;
    pid.kp = DEFAULT_KP;
    pid.ki = DEFAULT_KI;
    pid.kd = DEFAULT_KD;
    return pid;
}

inline motor_pid_t get_motor_pid(short motIdx, enum_state_t type) {
    return motors[motIdx].controller[type].pid;
}

bool update_motor_pid(short motIdx, enum_state_t type, motor_pid_t pid) {
    // Update value of pid
    motors[motIdx].controller[type].pid = pid;
    motors[motIdx].controller[type].kCoeffs[0] = Q15(motors[motIdx].controller[type].pid.kp);
    motors[motIdx].controller[type].kCoeffs[1] = Q15(motors[motIdx].controller[type].pid.ki);
    motors[motIdx].controller[type].kCoeffs[2] = Q15(motors[motIdx].controller[type].pid.kd);
    
    // TODO Add stop of all Interrupt during the time of this function
    // Derive the a, b and c coefficients from the Kp, Ki & Kd
    PIDCoeffCalc(&motors[motIdx].controller[type].kCoeffs[0], &motors[motIdx].controller[type].PIDstruct);
    // TODO add check gains value
    // | Kp + ki + kd | < 1
    // | -(Kp + 2*Kd) | < 1
    // | Kd | < 1
    return true;
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
    motors[motIdx].emergency_step = motors[motIdx].emergency.slope_time * DEFAULT_FREQ_MOTOR_MANAGER;
    motors[motIdx].emergency_stop = motors[motIdx].emergency.bridge_off * DEFAULT_FREQ_MOTOR_MANAGER;
    motors[motIdx].counter_alive = 0;
    motors[motIdx].counter_stop = 0;
}

inline motor_t get_motor_measures(short motIdx) {
    motors[motIdx].measure.position_delta = motors[motIdx].k_ang * motors[motIdx].PulsEnc;
    motors[motIdx].measure.position = motors[motIdx].enc_angle * motors[motIdx].k_ang; 
    motors[motIdx].measure.torque = motors[motIdx].diagnostic.current; //TODO Add a coefficient conversion
    motors[motIdx].PulsEnc = 0;
    return motors[motIdx].measure;
}

inline motor_diagnostic_t get_motor_diagnostic(short motIdx) {
    // Convert internal volt value in standard [mV]
    motors[motIdx].diagnostic.volt = motors[motIdx].volt.k * ((float) motors[motIdx].volt.value);
    // Convert internal current value in standard [mA]
    motors[motIdx].diagnostic.current = motors[motIdx].current.k * ((float) motors[motIdx].current.value);
    return motors[motIdx].diagnostic;
}

inline motor_t get_motor_reference(short motIdx) {
    return motors[motIdx].reference;
}
             
inline void reset_motor_position_measure(short motIdx, motor_control_t value) {
    motors[motIdx].enc_angle = (int)(((float)motors[motIdx].k_ang) / ((float)motors[motIdx].enc_angle));
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
        motors[motIdx].last_reference = motors[motIdx].reference.velocity;
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
        if(motors[motIdx].reference.state == CONTROL_EMERGENCY) {
            task_set(motors[motIdx].task_emergency, RUN);
        } else {
            task_set(motors[motIdx].task_emergency, STOP);
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
    //Measure velocity
    measureVelocity(motIdx);
    // Update current and volt values;
    // The current is evaluated with sign of motor rotation
    motors[motIdx].current.value = motors[motIdx].rotation * (gpio_get_analog(0, motors[motIdx].pin_current) - motors[motIdx].current.offset);
    motors[motIdx].volt.value = gpio_get_analog(0, motors[motIdx].pin_voltage) + motors[motIdx].volt.offset;
    
    
    fractional control_output = MotorPID(motIdx, &motors[motIdx].controller[GET_CONTROLLER_NUM(CONTROL_VELOCITY)].PIDstruct);
    
    // Send to motor the value of control
    Motor_PWM(motIdx, control_output);
}

void measureVelocity(short motIdx) {
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

        motors[motIdx].rotation = ((temp.SIG_VEL >= 0) ? 1 : -1);

        if (labs(vel_qei - vel_ic) < 2000) {
            // Mean velocity between IC and QEI estimation
            vel_mean = (vel_ic + vel_qei) / 2;
        } else {
            vel_mean = vel_qei;
        }
    } else {
        vel_mean = vel_qei;
    }
    // Store velocity
    motors[motIdx].measure.velocity = (motor_control_t) update_statistic(&motors[motIdx].mean_vel, vel_mean);
    //Select Prescaler
    motors[motIdx].prescaler_callback(motIdx);
    // Evaluate angle position
    if (labs(motors[motIdx].enc_angle) > motors[motIdx].angle_limit) {
        motors[motIdx].enc_angle = 0;
    }
}

inline void Motor_PWM(short motIdx, int pwm_control) {
    // Save pwm value
    motors[motIdx].measure.pwm = pwm_control;
    // PWM output
    pwm_control = motors[motIdx].parameter_motor.rotation * (pwm_control >> 4);
    SetDCMCPWM1(motIdx + 1, pwm_control + DEFAULT_PWM_OFFSET, 0);
}

inline fractional MotorPID(short motIdx, tPID *pid) {
    // Setpoint
    pid->controlReference = motors[motIdx].reference.velocity;
    // Measure
    pid->measuredOutput = motors[motIdx].measure.velocity;
    // PID execution
    PID(pid);
    // Control value calculation
    return pid->controlOutput;
}

void Emergency(int argc, int *argv) {
    short motIdx = (short) argv[0];
    if (motors[motIdx].reference.velocity != 0) {
        motors[motIdx].reference.velocity -= motors[motIdx].last_reference / (int16_t) (motors[motIdx].emergency_step + 0.5f);
        if (SGN(motors[motIdx].reference.velocity) * motors[motIdx].last_reference < 0) {
            motors[motIdx].reference.velocity = 0;
        }
    } else if (motors[motIdx].reference.velocity == 0) {
        if ((motors[motIdx].counter_stop + 1) >= motors[motIdx].emergency_stop) {
            set_motor_state(motIdx, CONTROL_DISABLE);
            motors[motIdx].counter_stop = 0;
        } else
            motors[motIdx].counter_stop++;
    }
}