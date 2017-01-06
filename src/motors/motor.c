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

#define MOTOR_DEFAULT_ADC_PORTS 2

#define DEFAULT_FREQ_MOTOR_MANAGER 1000.0             // Task Manager 10Khz
#define DEFAULT_FREQ_MOTOR_CONTROL_EMERGENCY 1000.0   // In Herts
#define DEFAULT_FREQ_MOTOR_CONTROL_SAFETY 1000.0      // In Herts

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

/******************************************************************************/
/* Communication Functions                                                    */
/******************************************************************************/

void Motor_TaskController(int argc, int *argv) {
    
}

void Motor_Emergency(int argc, int *argv) {
    
}

void Motor_Safety(int argc, int *argv) {
    
}

void Motor_ADC_callback(void* obj) {
    
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
void Motor_initialize_controllers(motor_firmware_t *motor, 
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

void Motor_init(motor_firmware_t *motor, unsigned int index, 
        fractional *abcCoefficient, fractional *controlHistory, 
        ICdata* ICinfo, event_prescaler_t prescaler_event, unsigned int PWM_LIMIT) {
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
        Motor_initialize_controllers(motor, &abcCoefficient[i], &controlHistory[i], i);
    }
    // Register input capture
    motor->ICinfo = ICinfo;
    motor->prescaler_callback = prescaler_event;
    // Set null ADC ports
    motor->adc = NULL;
    // Set null enable ports
    motor->pin_enable = NULL;
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

void Motor_register_adc(motor_firmware_t *motor, gpio_adc_t *adc) {
    /// Setup ADC current and temperature
    motor->adc = adc;
    gpio_adc_register(adc, MOTOR_DEFAULT_ADC_PORTS, &Motor_ADC_callback, motor);
}

void Motor_register_enable(motor_firmware_t *motor, const gpio_t* enable) {
    /// Setup bit enable
    motor->pin_enable = (gpio_t*) enable;
    gpio_init_pin(motor->pin_enable);
}

void Motor_run(motor_firmware_t *motor, task_status_t state) {
    task_set(motor->task_manager, state);
}