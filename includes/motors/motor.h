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
#include <or_peripherals/GPIO/led.h>

#include <or_math/statistics.h>

/******************************************************************************/
/* Prototype              													  */
/******************************************************************************/

#define NUM_CONTROLLERS 3

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
        // led controller
        LED_controller_t* led_controller;
        // Information for Input Capture
        ICdata* ICinfo;
        frequency_t ICfreq;
        // Enable pin for H-bridge
        gpio_t* pin_enable;
        gpio_adc_t *adc;
        float gain_adc;
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
/**
 * 
 * @param motor
 * @param index
 * @param abcCoefficient
 * @param controlHistory
 * @param ICinfo
 * @param ICfreq
 * @param prescaler_event
 * @param PWM_LIMIT
 */
void Motor_init(motor_firmware_t *motor, unsigned int index, 
        fractional *abcCoefficient, fractional *controlHistory, 
        ICdata* ICinfo, frequency_t ICfreq, event_prescaler_t prescaler_event, 
        unsigned int PWM_LIMIT);
/**
 * 
 * @param motor
 * @param adc
 * @param gain_adc
 */
void Motor_register_adc(motor_firmware_t *motor, gpio_adc_t *adc, float gain_adc);
/**
 * 
 * @param motor
 * @param enable
 */
void Motor_register_enable(motor_firmware_t *motor, const gpio_t* enable);
/**
 * 
 * @param motor
 * @param led_controller
 */
void Motor_register_led_controller(motor_firmware_t *motor, LED_controller_t* led_controller);
/**
 * 
 * @param motor
 * @param state
 */
void Motor_run(motor_firmware_t *motor, task_status_t state);
/**
 * 
 * @param motor
 * @return 
 */
inline motor_t Motor_get_constraints(motor_firmware_t *motor);
/**
 * 
 * @param motor
 * @param constraints
 */
void Motor_update_constraints(motor_firmware_t *motor, motor_t *constraints);
/**
 * 
 * @param motor
 * @return 
 */
inline motor_parameter_t Motor_get_parameters(motor_firmware_t *motor);
/**
 * 
 * @param motor
 * @param parameters
 */
void Motor_update_parameters(motor_firmware_t *motor, motor_parameter_t *parameters);
/**
 * 
 * @param motor
 * @return 
 */
inline motor_state_t Motor_get_state(motor_firmware_t *motor);
/**
 * 
 * @param motor
 * @param state
 */
void Motor_set_state(motor_firmware_t *motor, motor_state_t state);
/**
 * 
 * @param motor
 * @param state
 * @return 
 */
inline motor_pid_t Motor_get_pid(motor_firmware_t *motor, motor_state_t state);
/**
 * 
 * @param motor
 * @param state
 * @param pid
 * @return 
 */
bool Motor_update_pid(motor_firmware_t *motor, motor_state_t state, motor_pid_t *pid);
/**
 * 
 * @param motor
 * @return 
 */
inline motor_emergency_t Motor_get_emergency(motor_firmware_t *motor);
/**
 * 
 * @param motor
 * @param emergency_data
 */
void Motor_update_emergency(motor_firmware_t *motor, motor_emergency_t *emergency_data);
/**
 * 
 * @param motor
 * @return 
 */
inline motor_safety_t Motor_get_safety(motor_firmware_t *motor);
/**
 * 
 * @param motor
 * @param safety
 */
void Motor_update_safety(motor_firmware_t *motor, motor_safety_t *safety);
/**
 * 
 * @param motor
 * @return 
 */
inline motor_t Motor_get_measures(motor_firmware_t *motor);
/**
 * 
 * @param motor
 * @return 
 */
inline motor_t Motor_get_control(motor_firmware_t *motor);
/**
 * 
 * @param motor
 * @return 
 */
inline motor_diagnostic_t Motor_get_diagnostic(motor_firmware_t *motor);
/**
 * 
 * @param motor
 * @return 
 */
inline motor_t Motor_get_reference(motor_firmware_t *motor);
/**
 * 
 * @param motor
 * @param value
 */
inline void Motor_reset_position_measure(motor_firmware_t *motor, motor_control_t value);
/**
 * 
 * @param motor
 * @param state
 * @param reference
 */
void Motor_set_reference(motor_firmware_t *motor, motor_state_t state, motor_control_t reference);

#ifdef	__cplusplus
}
#endif

#endif	/* MOTOR_H */

