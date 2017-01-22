/*
 * Copyright (C) 2014-2017 Officine Robotiche
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

#include <stdio.h>
#include <string.h>

#include "motor/motor.h"

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

// Get the number in controller array from enum_state_t
#define GET_CONTROLLER_NUM(X) ((X) - 1)

#define mainMOTOR_TASK_PRIORITY				( tskIDLE_PRIORITY + 3 )

/* The check task may require a bit more stack as it calls sprintf(). */
#define mainMOTOR_TAKS_STACK_SIZE			( configMINIMAL_STACK_SIZE * 2 )
/* The execution period of the check task. */
#define mainMOTOR_TASK_PERIOD				( ( TickType_t ) 1000 / portTICK_PERIOD_MS )

/******************************************************************************/
/* System Level Functions                                                     */
/******************************************************************************/

static void vMotorTask(void *pvParameters) {
    // Load motor information
    MOTOR_t *motor = (MOTOR_t*) pvParameters;
    /* Used to wake the task at the correct frequency. */
    TickType_t xLastExecutionTime;

    /* Initialise xLastExecutionTime so the first call to vTaskDelayUntil()
    works correctly. */
    xLastExecutionTime = xTaskGetTickCount();

    for (;;) {
        /* Wait until it is time for the next cycle. */
        vTaskDelayUntil(&xLastExecutionTime, mainMOTOR_TASK_PERIOD);
        
        // TEMP
        LED_update(motor->Idx, 1);
    }

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

void Motor_Init(MOTOR_t *motor, fractional *abcCoefficient, fractional *controlHistory) {
    char buff[7];
    sprintf(buff, "Motor-%d", motor->Idx);
    /* Create the test tasks defined within this file. */
	xTaskCreate( vMotorTask, buff, mainMOTOR_TAKS_STACK_SIZE, motor, mainMOTOR_TASK_PRIORITY, NULL );
    
    unsigned int i;
    // Initialize all controllers
    for(i = 0; i < NUM_CONTROLLERS; i++) {
        Motor_initialize_controllers(motor, &abcCoefficient[3 * i], &controlHistory[3 * i], i);
    }
    
    LED_update(motor->Idx, 1);
}

/*------------------------ CONSTRAINTS ---------------------------------------*/
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
/*------------------------ PARAMETERS ----------------------------------------*/
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
        REGISTER_MASK_SET_HIGH(motor->qei.CONFIG, motor->qei.swap_mask);
    } else {
        REGISTER_MASK_SET_LOW(motor->qei.CONFIG, motor->qei.swap_mask);
    }
    // Configuration current and voltage gain
    // Convert gain current in [mA]
    motor->current.gain = (1000.0 * GAIN_ADC) / motor->parameter_motor.bridge.current_gain + 0.5f;
    motor->current.offset = (1000.0 * motor->parameter_motor.bridge.current_offset) / motor->parameter_motor.bridge.current_gain + 0.5f;
    // Convert gain volt in [mV]
    motor->volt.gain = 1000.0 * motor->parameter_motor.bridge.volt_gain * GAIN_ADC;
    motor->volt.offset = 1000.0 * motor->parameter_motor.bridge.volt_offset;
    // Setup state with new bridge configuration
//    Motor_set_state(motor, motor->state);
}

/*----------------------------------------------------------------------------*/
inline void Motor_IC_controller(MOTOR_t *motor, unsigned int newTime, int QEIDIR) {
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
    motor->qei.SIG_VEL += QEIDIR;
}

inline void Motor_IC_timer(MOTOR_t *motor) {
    motor->ICinfo.overTmr++; // timer overflow counter
}
