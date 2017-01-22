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

    /* Tasks must not attempt to return from their implementing
    function or otherwise exit.  In newer FreeRTOS port
        attempting to do so will result in an configASSERT() being
        called if it is defined.  If it is necessary for a task to
        exit then have the task call vTaskDelete( NULL ) to ensure
        its exit is clean. */
    //vTaskDelete( NULL );

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
