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

#ifndef MOTOR_H
#define	MOTOR_H

#ifdef	__cplusplus
extern "C" {
#endif

/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/
    
#include <stdint.h>        /* Includes uint16_t definition                    */
#include <stdbool.h>       /* Includes true/false definition                  */
#include <string.h>        /* Includes string defintions                      */
#include <dsp.h>           /* Includes DSP functions                          */
    
/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"
    
#include <or_bus/frame.h>  /* Include frame messages                          */
    
#include "peripherals/peripherals.h"
    
/******************************************************************************/
/* System Level #define Macros                                                */
/******************************************************************************/
    
#define NUM_CONTROLLERS 3

    typedef enum {
        // Motor disabled for high current
        CONTROL_SAFETY = -2,
        // Motor slow down to zero speed, then the bridge is turned off
        CONTROL_EMERGENCY = -1,
        // Motor disabled
        CONTROL_DISABLE = 0,
        // Motor controlled in position
        CONTROL_POSITION = 1,
        // Motor controlled in velocity
        CONTROL_VELOCITY = 2,
        // Motor controller in torque
        CONTROL_CURRENT = 3,
        // Motor controlled using direct PWM signals
        CONTROL_DIRECT = 4,
    } enum_state_t;
    
    typedef struct _icMode {
        // Configuration bit pre scaler Input capture
        short mode;
        // Gain of Input Capture mode
        short k;
    } ICMode_t;
    
    typedef struct _InputCapture {
        // List of Input capture modes
        const ICMode_t *ICmode;
        // Size of number of InputCapture mode change
        const size_t ICMode_size;
        // Time between first and second encoder pulse
        volatile unsigned int delta;
        // Gain multiplier multiplier according to IC scale
        volatile unsigned int k_mul;
        //Overflow timer
        volatile unsigned int overTmr;
        //Old time stored
        volatile unsigned int oldTime;
        //Time period from Input Capture
        volatile unsigned long timePeriod;
        //Mode of Input Capture
        volatile unsigned short number;
    } InputCapture_t;
#define MOTOR_IC_INIT(ICmode) {&(ICmode)[0], sizeof(ICMode) / sizeof(ICMode[0]), 0, 0, 0, 0, 0, 0}
    
#define MOTOR_QEI_SWAP_BIT_MASK BIT_MASK(8)
    
    typedef struct _QEI {
        // Configuration register
        REGISTER CONFIG;
        // Counter register
        REGISTER COUNTER;
        // Swap bit mask
        const unsigned int swap_mask;
        //Sign of versus rotation motor
        volatile int SIG_VEL;
    } QEI_t;
#define MOTOR_QEI_INIT(config, counter) {&(config), &(counter), MOTOR_QEI_SWAP_BIT_MASK, 0}
    
    /**
     * Definition of PID ecosystem
     */
    typedef struct _pid_control {
        // Message information about value of PID
        motor_pid_t pid;
        // Structure of PID from dsp library
        tPID PIDstruct;
        // Coefficients KP, KI, KD
        fractional kCoeffs[3];
        // Coefficient K anti wind up
        fractional k_aw;
        // enable
        volatile bool enable;
        // anti wind up correction
        volatile fractional saturation;
    } pid_controller_t;
#define MOTOR_INIT_CONTROLLER() {}
    
    typedef struct {
        // physical motor number definition
        const unsigned int Idx;
        // Input capture definition
        InputCapture_t ICinfo;
        // Quadrature Encoder Interface configuration
        QEI_t qei;
        // List of all PID controllers
        pid_controller_t controller[NUM_CONTROLLERS];
    } MOTOR_t;
    
#define MOTOR_INIT(Idx, ICmode, QEI) {Idx, MOTOR_IC_INIT(ICmode), QEI}
    
/******************************************************************************/
/* System Function Prototypes                                                 */
/******************************************************************************/

/**
 * Initialization Motor controller
 * @param motor The motor structure definition
 * @param abcCoefficient PID coefficient in DSP space
 * @param controlHistory PID history in DSP space
 */
void Motor_Init(MOTOR_t *motor, fractional *abcCoefficient, fractional *controlHistory);
/**
 * Evaluate the time between an encoder tick
 * @param motor The motor structure definition
 * @param newTime the value of Input capture register
 * @param QEIDIR the direction of encoder 
 *          [1 clockwise, 0 encoder without sign, -1 under clockwise]
 */
inline void Motor_IC_controller(MOTOR_t *motor, unsigned int newTime, int QEIDIR);
/**
 * Increase over flow timer in Input capture evaluation controller
 * @param motor The motor structure definition
 */
void Motor_IC_timer(MOTOR_t *motor);

#ifdef	__cplusplus
}
#endif

#endif	/* MOTOR_H */

