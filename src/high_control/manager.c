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

/* Device header file */
#if defined(__XC16__)
#include <xc.h>
#elif defined(__C30__)
#if defined(__dsPIC33E__)
#include <p33Exxxx.h>
#elif defined(__dsPIC33F__)
#include <p33Fxxxx.h>
#endif
#endif

#include <stdint.h>          /* For uint16_t definition                       */
#include <stdbool.h>         /* For true/false definition                     */
#include <dsp.h>
#include <string.h>
#include <float.h>

#include "system/system.h"

#include <peripherals/gpio.h>
#include <system/task_manager.h>

#include "high_control/manager.h"
#include "motors/motor_control.h"
#include "communication/serial.h"

#define HIGH_CONTROL "HIGH_CONTROL"
static string_data_t _MODULE_HIGH_CONTROL = {HIGH_CONTROL, sizeof(HIGH_CONTROL)};


static hTask_t HighControlTask = NULL;

//State controller
volatile motion_state_t control_state = 0;

#define MAX_HIGH_TASK 3

typedef unsigned short hHighControlTask_t;
typedef struct _control_task {
    bool autostart;
    motor_state_t state;
    control_task_init_t init;
    control_task_loop_t loop;
} control_task_manager_t;

control_task_manager_t high_level_task[MAX_HIGH_TASK];
unsigned short counter_task = 0;

unsigned int counter_odo = 0;

float sinTh_old = 0, cosTh_old = 1;
float wheel_m;

//Definition value for parameter unicycle
typedef struct parameter_unicycle_int {
    long radius_l;
    long radius_r;
    long wheelbase;
} parameter_unicycle_int_t;
parameter_unicycle_int_t parameter_unicycle_int;

motion_velocity_t reference, measure;
motion_parameter_unicycle_t parameter_unicycle;
motion_coordinate_t coordinate;

/*****************************************************************************/
/* Dead Reckoning functions                                                  */
/*****************************************************************************/

void reset_motion(void) {
    reference.v = 0;
    reference.w = 0;
    measure.v = 0;
    measure.w = 0;
}

bool add_task(bool autostart, control_task_init_t init, control_task_loop_t loop) {
    if(counter_task < MAX_HIGH_TASK) {
        high_level_task[counter_task].autostart = autostart;
        high_level_task[counter_task].init = init;
        high_level_task[counter_task].loop = loop;
        counter_task++;
        return true;
    } else
        return false;
}

bool load_all_task(void) {
    hHighControlTask_t hControl;
    /// Initialize all high level task
    if(counter_task > 0) {
        for(hControl = 0; hControl < MAX_HIGH_TASK; ++hControl) {
            /// Start function to initialize high level task
            high_level_task[hControl].init(&high_level_task[hControl].state);
            /// Set autostart to selected task
            if(high_level_task[hControl].autostart) {
                set_motion_state(hControl + 1);
            }
        }
        return true;
    }
    return false;
}

void HighControl_Init(void) {
    reset_motion();
    
    HighControlTask = task_load_data(register_event_p(register_module(&_MODULE_HIGH_CONTROL), &MotorTaskController, EVENT_PRIORITY_LOW), 10, 0, NULL);
    task_set(HighControlTask, load_all_task());
}

void HighLevelTaskController(int argc, int *argv) {
    /// Measure velocity unicycle
    VelocityMeasure();
    /// Odometry unicycle
    deadReckoning();
    
    /// High level task manager
    if (control_state == STATE_CONTROL_HIGH_VELOCITY) {
        /// Set led to velocity control
    } else if (control_state - 1 < counter_task) {
       set_motion_velocity_ref_unicycle(high_level_task[control_state - 1].loop(&measure, &coordinate));
    } else {
        set_motion_state(STATE_CONTROL_HIGH_DISABLE);
    }

#ifndef MOTION_CONTROL
    UpdateBlink(3, control_state);
#endif
}

motion_parameter_unicycle_t init_motion_parameter_unicycle(void) {
    motion_parameter_unicycle_t parameter_unicycle;
    parameter_unicycle.radius_l = 0.1;
    parameter_unicycle.radius_r = 0.1;
    parameter_unicycle.wheelbase = 0.1;
    return parameter_unicycle;
}

/* inline */ 
motion_parameter_unicycle_t get_motion_parameter_unicycle(void) {
    return parameter_unicycle;
}

void update_motion_parameter_unicycle(motion_parameter_unicycle_t parameter) {
    parameter_unicycle = parameter;
    parameter_unicycle_int.radius_l = ((int) (parameter_unicycle.radius_l * 1000.0));
    parameter_unicycle_int.radius_r = ((int) (parameter_unicycle.radius_r * 1000.0));
    parameter_unicycle_int.wheelbase = ((int) (parameter_unicycle.wheelbase * 1000.0));
    wheel_m = parameter_unicycle.wheelbase / 2;
}

motion_coordinate_t init_motion_coordinate(void) {
    motion_coordinate_t coordinate;
    coordinate.x = 0;
    coordinate.y = 0;
    coordinate.theta = 0;
    coordinate.space = 0;
    return coordinate;
}

inline motion_coordinate_t get_motion_coordinate(void) {
    return coordinate;
}

void update_motion_coordinate(motion_coordinate_t coord) {
    coordinate = coord;
    sinTh_old = sinf(coordinate.theta);
    cosTh_old = cosf(coordinate.theta);
}

/* inline */
motion_state_t get_motion_state(void) {
    return control_state;
}

void set_motion_state(motion_state_t state) {
    if (state != control_state) {
        control_state = state;
        if (control_state == STATE_CONTROL_HIGH_VELOCITY) {
            set_motor_state(MOTOR_LEFT, STATE_CONTROL_VELOCITY);
            set_motor_state(MOTOR_RIGHT, STATE_CONTROL_VELOCITY);
        } else if(control_state - 1 < counter_task) {
            set_motor_state(MOTOR_LEFT, high_level_task[control_state - 1].state);
            set_motor_state(MOTOR_LEFT, high_level_task[control_state - 1].state);
        } else {
            control_state = STATE_CONTROL_HIGH_DISABLE;
            set_motor_state(MOTOR_LEFT, STATE_CONTROL_EMERGENCY);
            set_motor_state(MOTOR_RIGHT, STATE_CONTROL_EMERGENCY);
        }
    }
}

inline motion_velocity_t get_motion_velocity_ref_unicycle(void) {
    return reference;
}

void set_motion_velocity_ref_unicycle(motion_velocity_t velocity) {
    reference = velocity;
    // >>>>> Second part: references calculation
    long int motor_left_refer = (long int) ((1.0f / parameter_unicycle.radius_l)*(velocity.v - (0.5f*parameter_unicycle.wheelbase * (velocity.w)))*1000);
    long int motor_right_refer = (long int) ((1.0f / parameter_unicycle.radius_r)*(velocity.v + (0.5f*parameter_unicycle.wheelbase * (velocity.w)))*1000);

    // >>>>> Saturation on 16 bit values
    if(motor_left_refer > INT16_MAX) {
        set_motor_reference(MOTOR_ZERO, CONTROL_VELOCITY, INT16_MAX);
    } else if (motor_left_refer < INT16_MIN) {
        set_motor_reference(MOTOR_ZERO, CONTROL_VELOCITY, INT16_MIN);
    } else {
        set_motor_reference(MOTOR_ZERO, CONTROL_VELOCITY, motor_left_refer);
    }
    if(motor_right_refer > INT16_MIN) {
        set_motor_reference(MOTOR_ONE, CONTROL_VELOCITY, INT16_MIN);
    } else if (motor_right_refer < INT16_MIN) {
        set_motor_reference(MOTOR_ONE, CONTROL_VELOCITY, INT16_MIN);
    } else {
        set_motor_reference(MOTOR_ONE, CONTROL_VELOCITY, motor_right_refer);
    }
    // <<<<< Saturation on 16 bit values
}

inline motion_velocity_t get_motion_velocity_meas_unicycle(void) {
    return measure;
}

int VelocityMeasure(void) {
    unsigned int t = TMR1; // Timing function
    long vel_v = (parameter_unicycle_int.radius_r * get_motor_measures(MOTOR_ONE).velocity + parameter_unicycle_int.radius_l * get_motor_measures(MOTOR_ZERO).velocity) / 2;
    long vel_w = (parameter_unicycle_int.radius_r * get_motor_measures(MOTOR_ONE).velocity - parameter_unicycle_int.radius_l * get_motor_measures(MOTOR_ZERO).velocity) / (2 * parameter_unicycle_int.wheelbase);
    measure.v = ((float) vel_v / 1000000);
    measure.w = ((float) vel_w / 1000);

    return TMR1 - t; // Time of execution
}

int deadReckoning(void) {
    unsigned int t = TMR1; // Timing function
    volatile motion_coordinate_t delta;
    float WheelSpL = parameter_unicycle.radius_l * get_motor_measures(MOTOR_ZERO).position;
    float WheelSpR = parameter_unicycle.radius_r * get_motor_measures(MOTOR_ONE).position;
    float SumSp = WheelSpR + WheelSpL; // Calcolo della somma degli spostamenti delle ruote
    float DifSp = WheelSpR - WheelSpL; // Calcolo della differenza degli spostamenti delle ruote

    if (fabs(DifSp) <= parameter_unicycle.sp_min) {
        delta.theta = 0;
        delta.space = WheelSpR;
        delta.x = delta.space * cosTh_old;
        delta.y = delta.space * sinTh_old;
    } else if (fabs(SumSp) <= parameter_unicycle.sp_min) {
        delta.theta = DifSp / parameter_unicycle.wheelbase;
        coordinate.theta = fmodf(coordinate.theta + delta.theta, 2 * PI); // Angolo normalizzato tra [0,2*PI]
        sinTh_old = sinf(coordinate.theta);
        cosTh_old = cosf(coordinate.theta);
        delta.x = 0;
        delta.y = 0;
        delta.space = 0;
    } else {
        delta.theta = DifSp / parameter_unicycle.wheelbase;
        coordinate.theta = fmodf(coordinate.theta + delta.theta, 2 * PI); // Angolo normalizzato tra [0,2*PI]
        float cosTh_new = cosf(coordinate.theta);
        float sinTh_new = sinf(coordinate.theta);
        delta.space = SumSp / 2;
        float radius = wheel_m * (SumSp / DifSp);

        delta.x = radius * (sinTh_new - sinTh_old);
        delta.y = radius * (cosTh_old - cosTh_new);
        sinTh_old = sinTh_new;
        cosTh_old = cosTh_new;
    }

    // Calculate odometry
    coordinate.space += delta.space;
    coordinate.x += delta.x;
    coordinate.y += delta.y;

    return TMR1 - t; // Time of execution
}