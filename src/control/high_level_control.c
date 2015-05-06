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

#include "control/high_level_control.h"
#include "control/motors.h"
#include "communication/serial.h"
#include "communication/parsing_messages.h"
#include "system/user.h"
#include "system/system.h"

//State controller
volatile state_controller_t control_state = 0;

unsigned int counter_odo = 0;
coordinate_t coordinate;
unsigned int counter_delta = 0;
bool autosend_delta_odometry = false;

float sinTh_old = 0, cosTh_old = 1;
float wheel_m;

//Definition value for parameter unicycle
typedef struct parameter_unicycle_int {
    long radius_l;
    long radius_r;
    long wheelbase;
} parameter_unicycle_int_t;
parameter_unicycle_int_t parameter_unicycle_int;

velocity_t vel_rif, vel_mis;

volatile parameter_unicycle_t parameter_unicycle;

//From system.c
extern process_t motion_process[PROCESS_MOTION_LENGTH];

/*****************************************************************************/
/* Dead Reckoning functions                                                  */
/*****************************************************************************/

void init_parameter_unicycle(void) {
    parameter_unicycle.radius_l = 0.04; //Radius left wheel
    parameter_unicycle.radius_r = 0.04; //Radius right wheel
    parameter_unicycle.wheelbase = 0.20; //Wheelbase
    parameter_unicycle.sp_min = 0.0001; // FLT_MIN
    update_parameter_unicycle();

    vel_mis.v = 0;
    vel_mis.w = 0;
}

void update_parameter_unicycle(void) {
    parameter_unicycle_int.radius_l = ((int) (parameter_unicycle.radius_l * 1000.0));
    parameter_unicycle_int.radius_r = ((int) (parameter_unicycle.radius_r * 1000.0));
    parameter_unicycle_int.wheelbase = ((int) (parameter_unicycle.wheelbase * 1000.0));
    wheel_m = parameter_unicycle.wheelbase / 2;
}

void init_coordinate(void) {
    coordinate.x = 0;
    coordinate.y = 0;
    coordinate.theta = 0;
    coordinate.space = 0;
}

void update_coord(void) {
    sinTh_old = sinf(coordinate.theta);
    cosTh_old = cosf(coordinate.theta);
}

void UpdateHighStateController(int state) {
    if (state != control_state) {
        control_state = state;
        switch (control_state) {
            case STATE_CONTROL_HIGH_DISABLE:
                set_motor_state(-1, STATE_CONTROL_DISABLE);
                break;
            default:
                set_motor_state(-1, STATE_CONTROL_VELOCITY);
                break;
        }
    }
}

int HighLevelTaskController(void) {
    unsigned int t = TMR1; // Timing function

    switch (control_state) {
        case STATE_CONTROL_HIGH_VELOCITY:
            /**
             * Measure linear and angular velocity for unicycle robot
             */
            VelocityMeasure();
            if (counter_odo >= motion_process[PROCESS_ODOMETRY].frequency) {
                motion_process[PROCESS_ODOMETRY].time = deadReckoning();
                counter_odo = 0;
            }
            counter_odo++;
            break;
        case STATE_CONTROL_HIGH_CONFIGURATION:
            break;
        default:
            set_motor_velocity(MOTOR_ZERO, 0);
            set_motor_velocity(MOTOR_ONE, 0);
            break;
    }
    return TMR1 - t; // Time of execution
}

int deadReckoning(void) {
    unsigned int t = TMR1; // Timing function
    volatile coordinate_t delta;
    float WheelSpL = parameter_unicycle.radius_l * get_motor_measures(MOTOR_ZERO).position;
    float WheelSpR = parameter_unicycle.radius_r * get_motor_measures(MOTOR_ONE).position;
    float SumSp = WheelSpR + WheelSpL; // Calcolo della somma degli spostamenti delle ruote
    float DifSp = WheelSpR - WheelSpL; // Calcolo della differenza degli spostamenti delle ruote
    //PulsEncL = 0; // Flush variabile
    //PulsEncR = 0; // Flush variabile

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
    odometry(delta);

    return TMR1 - t; // Time of execution
}

int odometry(coordinate_t delta) {
    unsigned int t = TMR1; // Timing function

    coordinate.space += delta.space;
    coordinate.x += delta.x;
    coordinate.y += delta.y;

    return TMR1 - t; // Time of esecution
}

int set_high_velocity(velocity_t velocity) {
    unsigned int t = TMR1; // Timing function

    vel_rif = velocity;
    // >>>>> References calculation
    long int motor_left_refer = (long int) ((1.0f / parameter_unicycle.radius_r)*(vel_rif.v + (parameter_unicycle.wheelbase * (-vel_rif.w)))*1000);
    long int motor_right_refer = (long int) ((1.0f / parameter_unicycle.radius_l)*(vel_rif.v - (parameter_unicycle.wheelbase * (-vel_rif.w)))*1000);

    // >>>>> Saturation on 16 bit values
    if(motor_left_refer > 32767) {
        set_motor_velocity(MOTOR_ZERO, 32767);
    } else if (motor_left_refer < -32768) {
        set_motor_velocity(MOTOR_ZERO, -32768);
    } else {
        set_motor_velocity(MOTOR_ZERO, motor_left_refer);
    }
    if(motor_right_refer > 32767) {
        set_motor_velocity(MOTOR_ONE, 32767);
    } else if (motor_right_refer < -32768) {
        set_motor_velocity(MOTOR_ONE, -32768);
    } else {
        set_motor_velocity(MOTOR_ONE, motor_right_refer);
    }
    // <<<<< Saturation on 16 bit values

    return TMR1 - t; // Time of execution
}

/* inline */ velocity_t get_high_velocity_ref(void) {
    return vel_rif;
}

int VelocityMeasure(void) {
    unsigned int t = TMR1; // Timing function
    long vel_v = (parameter_unicycle_int.radius_r * get_motor_measures(MOTOR_ONE).velocity + parameter_unicycle_int.radius_l * get_motor_measures(MOTOR_ZERO).velocity) / 2;
    long vel_w = (parameter_unicycle_int.radius_r * get_motor_measures(MOTOR_ONE).velocity - parameter_unicycle_int.radius_l * get_motor_measures(MOTOR_ZERO).velocity) / (2 * parameter_unicycle_int.wheelbase);
    vel_mis.v = ((float) vel_v / 1000000);
    vel_mis.w = ((float) vel_w / 1000);

    return TMR1 - t; // Time of execution
}