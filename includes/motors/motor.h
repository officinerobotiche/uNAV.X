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

#ifndef MOTOR_H
#define	MOTOR_H

#ifdef	__cplusplus
extern "C" {
#endif

/******************************************************************************/
/*	Include																	  */
/******************************************************************************/

#include <stdint.h>        /* Includes uint16_t definition                    */
#include <stdbool.h>       /* Includes true/false definition                  */
#include <dsp.h>

#include <or_bus/frame.h>
#include <or_system/soft_timer.h>
#include <or_system/task_manager.h>
#include <or_peripherals/GPIO/adc.h>

#include <or_math/statistics.h>

/******************************************************************************/
/* Prototype              													  */
/******************************************************************************/

#define NUM_CONTROLLERS 3
    // Get the number in controller array from enum_state_t
#define GET_CONTROLLER_NUM(X) ((X) - 1)

    typedef enum {
        CONTROL_SAFETY = -2, ///< Motor disabled for high current
        CONTROL_EMERGENCY = -1, ///< Motor slow down to zero speed, then the bridge is turned off
        CONTROL_DISABLE = 0, ///< Motor disabled
        CONTROL_POSITION = 1, ///< Motor controlled in position
        CONTROL_VELOCITY = 2, ///< Motor controlled in velocity
        CONTROL_CURRENT = 3, ///< Motor controller in torque
        CONTROL_DIRECT = 4, ///< Motor controlled using direct PWM signals
    } enum_state_t;

    typedef struct _ICdata {
        volatile unsigned int delta;
        volatile unsigned int k_mul; // k_vel multiplier according to IC scale
        volatile unsigned int overTmr; //Overflow timer
        volatile unsigned int oldTime; //Old time stored
        volatile unsigned long timePeriod; //Time period from Input Capture
        volatile int SIG_VEL; //Sign of versus rotation motor
        volatile unsigned short number; //Mode of Input Capture
    } ICdata;
    
    typedef void (*event_prescaler_t)(int motIdx);

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

    typedef struct _motor_firmware {
        unsigned int index;
        // Information for Input Capture
        ICdata* ICinfo;
        // Enable pin for H-bridge
        gpio_t* pin_enable;
        gpio_adc_t *adc;
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

    /******************************************************************************/
    /* System Function Prototypes                                                 */
    /******************************************************************************/

#ifdef	__cplusplus
}
#endif

#endif	/* MOTOR_H */

