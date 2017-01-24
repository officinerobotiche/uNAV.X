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

#include "motors/motor.h"
#include <or_math/math.h>

#define IC_TIMEPERIOD_TH_MAX 0x8000
#define IC_TIMEPERIOD_TH_MIN 2000

#define MOTOR_DEFAULT_ADC_PORTS 2
#define MOTOR_DEFAULT_LEVEL_LOCK 6

#define DEFAULT_FREQ_MOTOR_MANAGER 1000.0             // Task Manager 10Khz
#define DEFAULT_FREQ_MOTOR_CONTROL_EMERGENCY 1000.0   // In Herts
#define DEFAULT_FREQ_MOTOR_CONTROL_SAFETY 1000.0      // In Herts

// Get the number in controller array from enum_state_t
#define GET_CONTROLLER_NUM(X) ((X) - 1)

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

/******************************************************************************/
/* Communication Functions                                                    */
/******************************************************************************/
/**
 * Select the correct Input Capture pre scaler		
 * @param motIdx motor pointer reference
 */		
inline void Motor_SelectIcPrescaler(MOTOR_t *motor) {
    /** 
     * V = Kvel / timePeriod
     * is equal to:
     * timePeriod = Kvel / V = # Adimensional value
     * 
     * V -> inf , timePeriod -> 0   , ICmode -> 3 decrease pulses
     * V -> 0   , timePeriod -> inf , ICmode -> 0 increase pulses
     * 
     */
    int temp_number = 0;
    unsigned long doubletimePeriod = motor->ICinfo.delta;
    unsigned long halfPeriod = motor->ICinfo.delta;
    do {
        doubletimePeriod = doubletimePeriod * motor->ICinfo.ICmode[temp_number].k;
        halfPeriod = halfPeriod / motor->ICinfo.ICmode[temp_number].k;
        if (doubletimePeriod > IC_TIMEPERIOD_TH_MIN) {
            if (halfPeriod < IC_TIMEPERIOD_TH_MAX) {
                motor->ICinfo.k_mul = motor->ICinfo.ICmode[temp_number].k;
                return;
            }
        }
        motor->ICinfo.k_mul = motor->ICinfo.ICmode[temp_number].k;
        temp_number++;
    }while(temp_number <= (motor->ICinfo.ICMode_size - 1));
}

int32_t Motor_measureVelocity(MOTOR_t *motor) {
    volatile InputCapture_t temp;
    int QEICNTtmp = 0;
    int32_t vel_mean = 0;
    //Evaluate position
    QEICNTtmp = (int) motor->QEIinfo->COUNTER;
    *motor->QEIinfo->COUNTER = 0;
    
    motor->PulsEnc += QEICNTtmp;
    motor->enc_angle += QEICNTtmp;
    //Measure velocity from QEI
    int32_t vel_qei =  QEICNTtmp * motor->k_vel_qei;

    // Store timePeriod
    temp.timePeriod = motor->ICinfo.timePeriod;
    motor->ICinfo.timePeriod = 0;
    // Store value
    unsigned int SIG_VEL = motor->QEIinfo->SIG_VEL;
    motor->QEIinfo->SIG_VEL = 0;
    if (SIG_VEL) {
        temp.k_mul = motor->ICinfo.k_mul;
        // Evaluate velocity
        int32_t temp_f = temp.k_mul * motor->k_vel_ic;
        temp_f = temp_f / temp.timePeriod;
        int32_t vel_ic = SIG_VEL * temp_f;

        if (labs(vel_qei - vel_ic) < 2000) {
            // Mean velocity between IC and QEI estimation
            vel_mean = (vel_ic + vel_qei) / 2;
        } else {
            vel_mean = vel_qei;
        }
    } else {
        vel_mean = vel_qei;
    }
    //Select Pre scaler
    Motor_SelectIcPrescaler(motor);
    // Evaluate angle position
    if (labs(motor->enc_angle) > motor->angle_limit) {
        motor->enc_angle = 0;
    }
    return update_statistic(&motor->mean_vel, vel_mean);
}

#define SATURATION
inline int Motor_write_PWM(MOTOR_t *motor, int duty_cycle) {
    // Store the real value send to the PWM
    motor->reference.pwm = duty_cycle;
#ifdef SATURATION
    int error = 0;
    if(duty_cycle > motor->pwm_limit) {
        error = motor->pwm_limit - duty_cycle;
        duty_cycle = motor->pwm_limit;
    } else if(duty_cycle < (-motor->pwm_limit-1)) {
        error = (-motor->pwm_limit-1) - duty_cycle;
        duty_cycle = (-motor->pwm_limit-1);
    }
#else
    // Save PWM value with attenuation => K = 1 / 16
    duty_cycle = duty_cycle >> 4;
#endif
    // Real PWM send
    motor->measure.pwm = duty_cycle;
    // PWM output
    if(motor->state != CONTROL_DISABLE)
        gpio_pwm_set(motor->index + 1, motor->pwm_limit + ((motor_control_t) motor->parameter_motor.rotation) * duty_cycle, 0);
    else
        gpio_pwm_set(motor->index + 1, motor->pwm_limit, 0);
#ifdef SATURATION
    return error;
#else
    return 0;
#endif
}

inline __attribute__((always_inline)) int Motor_castToDSP(motor_control_t value, 
        motor_control_t constraint, volatile fractional *saturation) {
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

inline int __attribute__((always_inline)) Motor_control_velocity(MOTOR_t *motor, 
        motor_control_t reference, volatile fractional saturation) {
#define CONTROLLER_VEL GET_CONTROLLER_NUM(CONTROL_VELOCITY)
    // Set reference
    motor->controller[CONTROLLER_VEL].PIDstruct.controlReference = Motor_castToDSP(reference, motor->constraint.velocity, &motor->controller[CONTROLLER_VEL].saturation);
    motor->reference.velocity = motor->controller[CONTROLLER_VEL].PIDstruct.controlReference;
    // Set measure
    if(motor->measure.velocity > INT16_MAX) {
        motor->controller[CONTROLLER_VEL].PIDstruct.measuredOutput = INT16_MAX;
    } else if(motor->measure.velocity < INT16_MIN) {
        motor->controller[CONTROLLER_VEL].PIDstruct.measuredOutput = INT16_MIN;
    } else {
        motor->controller[CONTROLLER_VEL].PIDstruct.measuredOutput = motor->measure.velocity;
    }
    // Add anti wind up saturation from PWM
    // Add coefficient K_back calculation for anti wind up
    motor->controller[CONTROLLER_VEL].PIDstruct.controlOutput += 
            motor->controller[CONTROLLER_VEL].k_aw * saturation;
    // PID execution
    PID(&motor->controller[CONTROLLER_VEL].PIDstruct);
    // Set Output
    motor->controlOut.velocity = motor->controller[CONTROLLER_VEL].PIDstruct.controlOutput;
    return motor->controlOut.velocity;
#undef CONTROLLER_VEL
}

inline int __attribute__((always_inline)) Motor_control_current(MOTOR_t *motor, motor_control_t reference, volatile fractional saturation) {
#define CONTROLLER_CURR GET_CONTROLLER_NUM(CONTROL_CURRENT)
    // Set reference
    motor->controller[CONTROLLER_CURR].PIDstruct.controlReference = 
            Motor_castToDSP(reference, motor->constraint.current, 
            &motor->controller[CONTROLLER_CURR].saturation);
    motor->reference.current = motor->controller[CONTROLLER_CURR].PIDstruct.controlReference;
    // Set measure        
    if(motor->measure.current > INT16_MAX) {
        motor->controller[CONTROLLER_CURR].PIDstruct.measuredOutput = INT16_MAX;
    } else if(motor->measure.current < INT16_MIN) {
        motor->controller[CONTROLLER_CURR].PIDstruct.measuredOutput = INT16_MIN;
    } else {
        motor->controller[CONTROLLER_CURR].PIDstruct.measuredOutput = motor->measure.current;
    }
    // Add anti wind up saturation from PWM
    // Add coefficient K_back calculation for anti wind up
    motor->controller[CONTROLLER_CURR].PIDstruct.controlOutput += 
            motor->controller[CONTROLLER_CURR].k_aw * saturation;
    // PID execution
    PID(&motor->controller[CONTROLLER_CURR].PIDstruct);
    // Get Output
    return motor->controller[CONTROLLER_CURR].PIDstruct.controlOutput;
#undef CONTROLLER_CURR
}

void Motor_TaskController(int argc, int *argv) {
    MOTOR_t *motor = (MOTOR_t*) argv[0];
    
    // ================ SAFETY CHECK ===========================
    /// Check for emergency mode or in safety mode
    if (motor->state > CONTROL_DISABLE) {
        // Check safety condition
        if (labs(motor->measure.current) > motor->safety.critical_zone) {
            // Run timer if current is too high
            if (run_timer(&motor->motor_safety.stop)) {
                // Store old state
                motor->motor_safety.old_state = motor->state;
                // Change motor state
                Motor_set_state(motor, CONTROL_SAFETY);
            }
        } else {
            // Reset the counter if the value is low
            reset_timer(&motor->motor_safety.stop);
        }
    }
    // Check emergency 
    if(motor->state != CONTROL_EMERGENCY && motor->state != CONTROL_DISABLE) {
        // check to run emergency mode
        if(run_timer(&motor->motor_emergency.alive)) {
            /// Set Motor in emergency mode
            Motor_set_state(motor, CONTROL_EMERGENCY);
            // Reset stop timer
            reset_timer(&motor->motor_emergency.stop);
        }
    }
    // ================ RUN CONTROLLERS ========================
    /// Add new task controller
    if(motor->state != motor->diagnostic.state) {
        if(motor->state == CONTROL_EMERGENCY) {
            if(motor->diagnostic.state == CONTROL_SAFETY) {
                // Stop safety task
                task_set(motor->motor_safety.task_safety, STOP);
            }
            task_set(motor->task_emergency, RUN);
        } else {
            task_set(motor->task_emergency, STOP);
        }
        if(motor->state == CONTROL_SAFETY) {
            task_set(motor->motor_safety.task_safety, RUN);
        }
        /// Save new state controller
        motor->diagnostic.state = motor->state;
    }
    // ================ READ MEASURES ==========================
    
    bool velocity_control = run_timer(&motor->controller[GET_CONTROLLER_NUM(CONTROL_VELOCITY)].timer);
    
    // Check if is the time to run the controller
    if (velocity_control) {
        //Measure velocity in milli rad/s
        motor->measure.velocity = (motor_control_t) Motor_measureVelocity(motor);
    }
    
    // ================ RUN CONTROLLERS ==========================
    
    // If some controller is selected
    if (motor->state != CONTROL_DISABLE) {
        // ========== CONTROL DIRECT =============
        if(motor->state == CONTROL_DIRECT) {
            // Send to motor the value of control
            Motor_write_PWM(motor, motor->external_reference);
            return;
        }
        // ========= CONTROL VELOCITY ============
#define ENABLE_VELOCITY_CONTROL
        // Check if the velocity control is enabled
        if (motor->controller[GET_CONTROLLER_NUM(CONTROL_VELOCITY)].enable) {
            // Check if is the time to run the controller
            if (velocity_control) {
                // Run PID control
                volatile fractional saturation = 0;
                if(motor->controller[GET_CONTROLLER_NUM(CONTROL_CURRENT)].enable == true) {
                    saturation = motor->controller[GET_CONTROLLER_NUM(CONTROL_CURRENT)].saturation;
                } else {
                    saturation = motor->pwm_saturation;
                }
#ifdef ENABLE_VELOCITY_CONTROL
                motor->control_output = Motor_control_velocity(motor, motor->external_reference, saturation);
#else
                Motor_control_velocity(motor, motor->external_reference, saturation);
                motor->control_output = 0;
#endif
            }
        } else {
            // Set the reference for the other current control the same of the reference
            motor->control_output = motor->external_reference;
        }
        // =======================================

#ifndef CURRENT_CONTROL_IN_ADC_LOOP
        // ========= CONTROL CURRENT =============
        // Check if the current control is enabled
        if (motor->controller[GET_CONTROLLER_NUM(CONTROL_CURRENT)].enable) {
            // Check if is the time to run the controller
            if (run_controller(&motor->controller[GET_CONTROLLER_NUM(CONTROL_CURRENT)])) {
                // The current is evaluated with sign of motor rotation
                // Update current and volt values;
                motor->measure.current = -motor->current.gain * motor->adc[0].value;
                        + motor->current.offset;
                motor->diagnostic.volt = motor->volt.gain * motor->adc[1].value;
                        + motor->volt.offset;
                // Run PID control
                motor->control_output = Motor_control_current(motor, motor->external_reference, motor->pwm_saturation);

                // Send to motor the value of control
                Motor_write_PWM(motor, motor->parameter_motor.rotation * motor->control_output);
            }
        }
        // =======================================
#else
        // If disabled Send the PWM after this line
        if (motor->controller[GET_CONTROLLER_NUM(CONTROL_CURRENT)].enable) {
            // Lock ADC current loop controller
            spin_lock(&motor->lock, MOTOR_DEFAULT_LEVEL_LOCK);
            // Set current reference
            motor->controller[GET_CONTROLLER_NUM(CONTROL_CURRENT)].PIDstruct.controlReference = 
                    Motor_castToDSP(motor->control_output, motor->constraint.current,
                    &motor->controller[GET_CONTROLLER_NUM(CONTROL_CURRENT)].saturation);
            motor->reference.current = 
                    motor->controller[GET_CONTROLLER_NUM(CONTROL_CURRENT)].PIDstruct.controlReference;
            // Unlock ADC current loop controller
            spin_unlock(&motor->lock);
        } else {
            // Send to motor the value of control
            motor->pwm_saturation = Motor_write_PWM(motor, motor->control_output);
        }
#endif
    }
}

void Motor_Emergency(int argc, int *argv) {
    MOTOR_t *motor = (MOTOR_t*) argv[0];
    if (motor->external_reference != 0) {
        motor->external_reference -= motor->last_reference / motor->motor_emergency.step;
        if (SGN(motor->external_reference) * motor->last_reference < 0) {
            motor->external_reference = 0;
        }
    } else if (motor->external_reference == 0) {
        if(run_timer(&motor->motor_emergency.stop)) {
            Motor_set_state(motor, CONTROL_DISABLE);
        }
    }
}

void Motor_restore_safety_control(MOTOR_t *motor) {
    // Reset stop time
    reset_timer(&motor->motor_safety.stop);
    // Stop safety controller
    task_set(motor->motor_safety.task_safety, STOP);
}

void Motor_Safety(int argc, int *argv) {
    MOTOR_t *motor = (MOTOR_t*) argv[0];
    // Reduction external reference
    if(labs(motor->measure.current) > motor->safety.critical_zone) {
        if (motor->external_reference != 0) {
            motor->external_reference -= motor->last_reference / motor->motor_safety.step;
            if (SGN(motor->external_reference) * motor->last_reference < 0) {
                motor->external_reference = 0;
            }
        }
        // Reset recovery time
        reset_timer(&motor->motor_safety.restore);
    } else {
        // Check if auto restore is disabled
        if(motor->safety.autorestore != 0) {
            // Run auto recovery timer
            if(run_timer(&motor->motor_safety.restore)) {
                // Restore control
                Motor_restore_safety_control(motor);
                // Restore old controller
                Motor_set_state(motor, motor->motor_safety.old_state);
            }
        }
    }
}

void Motor_ADC_callback(void* obj) {
    MOTOR_t *motor = (MOTOR_t*) obj;
#define CONTROLLER_CURR GET_CONTROLLER_NUM(CONTROL_CURRENT)
    // Lock all events
    lock(&motor->lock, true);
    motor->measure.current = - motor->current.gain * motor->adc[0].value + motor->current.offset;
    motor->diagnostic.volt = motor->volt.gain * motor->adc[1].value + motor->volt.offset;
    
    if(motor->controller[CONTROLLER_CURR].enable) {
        // Set measure        
        if(motor->measure.current > INT16_MAX) {
            motor->controller[CONTROLLER_CURR].PIDstruct.measuredOutput = INT16_MAX;
        } else if(motor->measure.current < INT16_MIN) {
            motor->controller[CONTROLLER_CURR].PIDstruct.measuredOutput = INT16_MIN;
        } else {
            motor->controller[CONTROLLER_CURR].PIDstruct.measuredOutput = motor->measure.current;
        }
        // Add anti wind up saturation from PWM
        // Add coefficient K_back calculation for anti wind up
        motor->controller[CONTROLLER_CURR].PIDstruct.controlOutput += 
                motor->controller[CONTROLLER_CURR].k_aw * motor->pwm_saturation;
        // PID execution
        PID(&motor->controller[CONTROLLER_CURR].PIDstruct);
        // Set Output        
        motor->pwm_saturation = Motor_write_PWM(motor, motor->controller[CONTROLLER_CURR].PIDstruct.controlOutput);
    }
    // Unlock the controller
    lock(&motor->lock, false);
#undef CONTROLLER_CURR
}

/**
 * @brief Reset Motor values
 * @param motor The motor controller structure
 */
void Motor_reset_data(volatile motor_t* motor) {
    motor->position_delta = 0;
    motor->position = 0;
    motor->velocity = 0;
    motor->current = 0;
    motor->effort = 0;
    motor->pwm = 0;
}
/**
 * @brief Initialization PID controllers
 * @param motor The motor controller structure
 * @param abcCoefficient ABC coefficient for PID
 * @param controlHistory History structure for PID
 * @param i The number of controller
 */
void Motor_initialize_controllers(MOTOR_t *motor, 
        fractional *abcCoefficient, fractional *controlHistory, unsigned int i) {
    //Initialize the PID data structure: PIDstruct
    //Set up pointer to derived coefficients
    motor->controller[i].PIDstruct.abcCoefficients = abcCoefficient;
    //Set up pointer to controller history samples
    motor->controller[i].PIDstruct.controlHistory = controlHistory;
    // Clear the controller history and the controller output
    PIDInit(&motor->controller[i].PIDstruct);
    // Enable
    motor->controller[i].enable = false;
    // Saturation value
    motor->controller[i].saturation = 0;
    // Gain anti wind up
    motor->controller[i].k_aw = 0;
}

void Motor_init(MOTOR_t *motor, unsigned int index, fractional *abcCoefficient, 
        fractional *controlHistory, unsigned int PWM_LIMIT) {
    unsigned int i;
    // Set index number
    motor->index = index;
    // reset motor values
    Motor_reset_data(&motor->measure);
    Motor_reset_data(&motor->reference);
    Motor_reset_data(&motor->controlOut);
    // PWM limit
    motor->pwm_limit = PWM_LIMIT;
    // Initialize all controllers
    for(i = 0; i < NUM_CONTROLLERS; i++) {
        Motor_initialize_controllers(motor, &abcCoefficient[3 * i], &controlHistory[3 * i], i);
    }
    // Set null ADC ports
    motor->adc = NULL;
    // Set null enable ports
    motor->pin_enable = NULL;
    // Set NULL led controller
    motor->led_controller = NULL;
    // reset values
    motor->control_output = 0;
    motor->external_reference = 0;
    motor->pwm_saturation = 0;
    //Setup diagnostic
    motor->diagnostic.time_control = 0;
    motor->diagnostic.watt = 0;
    motor->diagnostic.temperature = 0;
    motor->diagnostic.time_control = 0;
    motor->diagnostic.state = CONTROL_DISABLE;
    // Initialize mean buffer
    init_statistic_buffer(&motor->mean_vel);
    
    // Setup frequency task manager
    motor->manager_freq = DEFAULT_FREQ_MOTOR_MANAGER;
    /// Register event and add in task controller - Working at 1KHz
    motor->motor_manager_event = register_event_p(&Motor_TaskController, EVENT_PRIORITY_MEDIUM);
    motor->task_manager = task_load_data(motor->motor_manager_event, motor->manager_freq, 1, motor);
    /// Load controller EMERGENCY - Working at 1KHz
    hEvent_t emergency_event = register_event_p(&Motor_Emergency, EVENT_PRIORITY_HIGH);
    motor->task_emergency = task_load_data(emergency_event, DEFAULT_FREQ_MOTOR_CONTROL_EMERGENCY, 1, motor);
    /// Load controller SAFETY - Working at 1KHz
    hEvent_t safety_event = register_event_p(&Motor_Safety, EVENT_PRIORITY_HIGH);
    motor->motor_safety.task_safety = task_load_data(safety_event, DEFAULT_FREQ_MOTOR_CONTROL_SAFETY, 1, motor);
}

void Motor_register_QEI_IC(MOTOR_t *motor, QEI_t *qei,
        const ICMode_t *ICMode, size_t ICMode_size, 
        unsigned short default_num, frequency_t ICfreq) {
    motor->QEIinfo = qei;
    motor->QEIinfo->SIG_VEL = 0;
    // Register input capture
    motor->ICfreq = ICfreq;
    // Reset all values
    motor->ICinfo.overTmr = 0;
    motor->ICinfo.oldTime = 0;
    motor->ICinfo.timePeriod = 0;
    //Input capture information
    motor->ICinfo.ICmode = ICMode;
    motor->ICinfo.ICMode_size = ICMode_size;
    motor->ICinfo.k_mul = motor->ICinfo.ICmode[default_num].k;
    motor->ICinfo.number = default_num;
}

void Motor_register_adc(MOTOR_t *motor, gpio_adc_t *adc, float gain_adc) {
    /// Setup ADC current and temperature
    motor->adc = adc;
    motor->gain_adc = gain_adc;
    gpio_adc_register(adc, MOTOR_DEFAULT_ADC_PORTS, &Motor_ADC_callback, motor);
}

void Motor_register_enable(MOTOR_t *motor, gpio_t* enable) {
    /// Setup bit enable
    motor->pin_enable = enable;
    gpio_init_pin(motor->pin_enable);
}

void Motor_register_led_controller(MOTOR_t *motor, LED_controller_t* led_controller) {
    motor->led_controller = led_controller;
}

void Motor_run(MOTOR_t *motor, task_status_t state) {
    task_set(motor->task_manager, state);
}

inline motor_t Motor_get_constraints(MOTOR_t *motor) {
    return motor->constraint;
}

void Motor_update_constraints(MOTOR_t *motor, motor_t *constraints) {
    //Update parameter constraints
    memcpy(&motor->constraint, constraints, sizeof(motor_t));
    //Update PWM max value
    //Shifted from maximum motor_control_t value to maximum PWM value
    // 31bit -> 11 bit = 20
    motor->pwm_limit = motor->constraint.pwm >> 20;
}

inline motor_parameter_t Motor_get_parameters(MOTOR_t *motor) {
    return motor->parameter_motor;
}

void Motor_update_parameters(MOTOR_t *motor, motor_parameter_t *parameters) {
    //Update parameter configuration
    memcpy(&motor->parameter_motor, parameters, sizeof(motor_parameter_t));
    // If CPR is before ratio
    //    ThC = CPR * RATIO   
    // else
    //    ThC = RATIO
    uint32_t angle_ratio;
    if(motor->parameter_motor.encoder.type.position) {
        angle_ratio = motor->parameter_motor.encoder.cpr * ((uint32_t) 1000 * motor->parameter_motor.ratio);
    } else {
        angle_ratio = (uint32_t) 1000 * motor->parameter_motor.encoder.cpr;
    }
    // Evaluate angle limit
    motor->angle_limit = (angle_ratio * 4) / 1000;
    //Start define with fixed K_vel conversion velocity
    // KVEL = FRTMR2 *  [ 2*pi / ( ThC * 2 ) ] * 1000 (velocity in milliradiant)
    motor->k_vel_ic = 1000000.0f * motor->ICfreq * 2 * PI / (angle_ratio * 2);
    //Start define with fixed K_ang conversion angular
    // K_ANG = 2*PI / ( ThC * (QUADRATURE = 4) )
    motor->k_ang = (float) 2000.0f *PI / (angle_ratio * 4);
    // vel_qei = 1000 * QEICNT * Kang / Tc = 1000 * QEICNT * Kang * Fc
    //motors[motIdx].k_vel_qei = (motors[motIdx].k_ang * 1000.0f * motors[motIdx].manager_freq);
    motor->k_vel_qei = (motor->k_ang * 1000.0f * ((float) motor->controller[GET_CONTROLLER_NUM(CONTROL_VELOCITY)].pid.frequency));
    // Set Swap bit
    // Phase A and Phase B inputs swapped
    if(motor->parameter_motor.rotation >= 1) {
        REGISTER_MASK_SET_HIGH(motor->QEIinfo->swap.REG, motor->QEIinfo->swap.CS_mask);
    } else {
        REGISTER_MASK_SET_LOW(motor->QEIinfo->swap.REG, motor->QEIinfo->swap.CS_mask);
    }
    // Configuration current and voltage gain
    if (motor->adc != NULL) {
        // Convert gain current in [mA]
        motor->current.gain = (1000.0 * motor->gain_adc) / motor->parameter_motor.bridge.current_gain + 0.5f;
        motor->current.offset = (1000.0 * motor->parameter_motor.bridge.current_offset) / motor->parameter_motor.bridge.current_gain + 0.5f;
        // Convert gain volt in [mV]
        motor->volt.gain = 1000.0 * motor->parameter_motor.bridge.volt_gain * motor->gain_adc;
        motor->volt.offset = 1000.0 * motor->parameter_motor.bridge.volt_offset;
    }
    // Setup state with new bridge configuration
    Motor_set_state(motor, motor->state);
}

inline motor_state_t Motor_get_state(MOTOR_t *motor) {
    return motor->diagnostic.state;
}

void Motor_set_state(MOTOR_t *motor, motor_state_t state) {
    bool enable = (state == CONTROL_DISABLE) ? false : true;
    // For all error controller the blinks are disabled
    int led_state = (state < CONTROL_DISABLE) ? state : state + 1;
    
    // Restore safety control when auto restore is disabled
    if(motor->safety.autorestore == 0 && motor->state == CONTROL_SAFETY && state != CONTROL_SAFETY) {
        Motor_restore_safety_control(motor);
    }
    
    /// Set enable or disable motors
    motor->state = state;
    if(motor->pin_enable != NULL) {
        if(enable == (motor->parameter_motor.bridge.enable == MOTOR_ENABLE_LOW)) {
            gpio_set_pin(motor->pin_enable, GPIO_HIGH);
        } else {
            gpio_set_pin(motor->pin_enable, GPIO_LOW);
        }
    }
    // Enable or disable PWM controller
    bool enable_pwm = (state != CONTROL_DISABLE) ? true : false;
    gpio_pwm_enable(motor->index, enable_pwm);
    // Store last velocity if run some error mode
    if (state < CONTROL_DISABLE) {
        motor->last_reference = motor->reference.velocity;
    }
    // Update LED blink if registered
    if(motor->led_controller != NULL) {
        LED_updateBlink(motor->led_controller, motor->index, led_state);
    }
}

inline motor_pid_t Motor_get_pid(MOTOR_t *motor, motor_state_t state) {
    int num_control = GET_CONTROLLER_NUM(state);
    return motor->controller[num_control].pid;
}

bool Motor_update_pid(MOTOR_t *motor, motor_state_t state, motor_pid_t *pid) {
    // If the motor control is the same in action you can not disable
    if(motor->diagnostic.state == state && pid->enable == false) {
        return false;
    }
    // Check gains value
    // Check1 = | Kp + ki + kd | < 1 = INT16_MAX
    // Check2 = | -(Kp + 2*Kd) | < 1 = INT16_MAX
    // Check3 =      | Kd |      < 1 = INT16_MAX
    long check1 = labs(1000.0 * (pid->kp + pid->ki + pid->kd));
    long check2 = labs(1000.0 * (pid->kp + 2 * pid->kd));
    long check3 = labs(1000.0 * pid->kd);
    
    if(check1 < INT16_MAX && check2 < INT16_MAX && check3 < INT16_MAX) {
        spin_lock(&motor->lock, MOTOR_DEFAULT_LEVEL_LOCK);
        int num_control = GET_CONTROLLER_NUM(state);
        // Update PID struct
        memcpy(&motor->controller[num_control].pid, pid, sizeof(motor_pid_t));
        // Write new coefficients
        motor->controller[num_control].kCoeffs[0] = (int) (pid->kp * 1000.0);
        motor->controller[num_control].kCoeffs[1] = (int) (pid->ki * 1000.0);
        motor->controller[num_control].kCoeffs[2] = (int) (pid->kd * 1000.0);
        // Derive the a, b and c coefficients from the Kp, Ki & Kd
        PIDCoeffCalc(&motor->controller[num_control].kCoeffs[0], 
                &motor->controller[num_control].PIDstruct);
        // Clear the controller history and the controller output
        PIDInit(&motor->controller[num_control].PIDstruct);
        // Gain anti wind up
        motor->controller[num_control].k_aw = (int) (pid->kaw * 1000.0);
#ifdef CURRENT_CONTROL_IN_ADC_LOOP
        if(state == CONTROL_CURRENT) {
            // Manual reset frequency  current loop
            motor->controller[num_control].pid.frequency = CURRENT_ADC_LOOP_FRQ;
        } else {
            // Initialize soft timer
            init_soft_timer(&motor->controller[num_control].timer, motor->manager_freq, 1000000 / pid->frequency);
        }
#else
        // Initialize soft timer
        init_soft_timer(&motor->controller[num_control].timer, motor->manager_freq, 1000000 / pid->frequency);
#endif
        // Set enable PID
        motor->controller[num_control].enable = pid->enable;
        // Update K_QEI (Quadrature Encoder Interface)
        if(state == CONTROL_VELOCITY) {
            motor->k_vel_qei = motor->k_ang * 1000.0f *((float) pid->frequency);
        }
        spin_unlock(&motor->lock);
        return true;
    } else
        return false;
}

inline motor_emergency_t Motor_get_emergency(MOTOR_t *motor) {
    return motor->emergency;
}

void Motor_update_emergency(MOTOR_t *motor, motor_emergency_t *emergency_data) {
    // Store emergency data
    memcpy(&motor->emergency, emergency_data, sizeof(motor_emergency_t));
    // Reset counter alive reference message
    init_soft_timer(&motor->motor_emergency.alive, (motor->manager_freq / 1000), emergency_data->timeout * 1000000);
    // Reset counter Emergency stop
    init_soft_timer(&motor->motor_emergency.stop, motor->manager_freq, motor->emergency.bridge_off * 1000000);
    // Fixed step to slow down the motor
    motor->motor_emergency.step = motor->emergency.slope_time * motor->manager_freq;
}

inline motor_safety_t Motor_get_safety(MOTOR_t *motor) {
    return motor->safety;
}

void Motor_update_safety(MOTOR_t *motor, motor_safety_t *safety) {
    // Store safety data
    memcpy(&motor->safety, safety, sizeof(motor_safety_t));
    // Initialization safety controller
    init_soft_timer(&motor->motor_safety.stop, (motor->manager_freq / 1000), safety->timeout * 1000000);
    // Initialization auto recovery system
    init_soft_timer(&motor->motor_safety.restore, (motor->manager_freq / 1000), safety->autorestore * 1000000);
    // Fixed step to slow down the motor in function of stop time
    motor->motor_safety.step = (safety->timeout * motor->manager_freq) / 1000;
}

inline motor_t Motor_get_measures(MOTOR_t *motor) {
    motor->measure.position_delta = motor->k_ang * motor->PulsEnc;
    motor->measure.position = motor->k_ang * motor->enc_angle;
    motor->measure.current = motor->parameter_motor.rotation * motor->measure.current;
    // Torque in [m Nm]
    motor->measure.effort = (motor->measure.current * motor->diagnostic.volt) / motor->measure.velocity;
    motor->PulsEnc = 0;
    return motor->measure;
}

inline motor_t Motor_get_control(MOTOR_t *motor) {
    motor->controlOut.current = motor->controller[GET_CONTROLLER_NUM(CONTROL_CURRENT)].PIDstruct.controlOutput;
    return motor->controlOut;
}

inline motor_diagnostic_t Motor_get_diagnostic(MOTOR_t *motor) {
    // get time execution
    motor->diagnostic.time_control = get_time(motor->motor_manager_event);
    // Evaluate the Watt required [mW]
    motor->diagnostic.watt = (motor->measure.current * motor->diagnostic.volt) / 1000;
    return motor->diagnostic;
}

inline motor_t Motor_get_reference(MOTOR_t *motor) {
    return motor->reference;
}
             
inline void Motor_reset_position_measure(MOTOR_t *motor, motor_control_t value) {
    motor->enc_angle = (int)((value / ((float)motor->k_ang) ));
    motor->measure.position = (float) value;  
}
             
void Motor_set_reference(MOTOR_t *motor, motor_state_t state, motor_control_t reference) {
    // If the controller is in safety state the new reference is skipped
    if(motor->diagnostic.state != CONTROL_SAFETY) {
        // Check state
        if(state != motor->state) {
            // Change motor control type
            Motor_set_state(motor, state);
            // Update reference
            motor->state = state;
        }
        // Setup reference
        motor->external_reference = reference;
    }
    // Reset time emergency
    reset_timer(&motor->motor_emergency.alive);
}

inline void Motor_IC_controller(MOTOR_t *motor, REGISTER ICBUF, bool QEIDIR) {
    unsigned int newTime = *ICBUF; // Reading ICBUF every interrupt
    
    // Detail in Microchip Application Note: AN545
    if(motor->ICinfo.overTmr == 0) {
        motor->ICinfo.delta = newTime - motor->ICinfo.oldTime;
        motor->ICinfo.timePeriod += motor->ICinfo.delta;
    } else {
        motor->ICinfo.delta = (newTime + (0xFFFF - motor->ICinfo.oldTime)
                + (0xFFFF * (motor->ICinfo.overTmr - 1)));
        motor->ICinfo.timePeriod += motor->ICinfo.delta;
        motor->ICinfo.overTmr = 0;
    }
    // Store old time period
    motor->ICinfo.oldTime = newTime;
    
    /// Save sign rotation motor
    (QEIDIR ? motor->QEIinfo->SIG_VEL++ : motor->QEIinfo->SIG_VEL--); 
}

inline void Motor_IC_timer(MOTOR_t *motor) {
    motor->ICinfo.overTmr++; // timer overflow counter
}