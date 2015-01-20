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
#include "control/motors_PID.h"
#include "communication/serial.h"
#include "communication/parsing_messages.h"
#include "system/user.h"

//State controller
volatile state_controller_t control_state = 0;

coordinate_t coordinate;
//delta_odometry_t delta_odometry;
unsigned int counter_delta = 0;
bool autosend_delta_odometry = false;

float sinTh_old = 0, cosTh_old = 1;

//Definition value for parameter unicycle

typedef struct parameter_unicycle_int {
    long radius_l;
    long radius_r;
    long wheelbase;
} parameter_unicycle_int_t;
parameter_unicycle_int_t parameter_unicycle_int;

velocity_t vel_rif, vel_mis;

//variables for emergency
volatile parameter_unicycle_t parameter_unicycle;
emergency_t emergency;
velocity_t last_vel_rif;
bool save_velocity = true;

// From motors PID
extern motor_control_t motor_ref[NUM_MOTORS];
extern unsigned int control_motor_state[NUM_MOTORS];
extern parameter_motor_t parameter_motor_left, parameter_motor_right;
extern motor_t motor_left, motor_right;
extern volatile int PulsEncL, PulsEncR;
extern k_odo_t k_odo;
extern float wheel_m;

//From System
extern parameter_system_t parameter_system;

//From interrupt
extern unsigned int counter_stop;

/******************************************************************************/
/* Dead Reckoning functions                                                   */

/******************************************************************************/

void init_parameter_unicycle(void) {
    parameter_unicycle.radius_l = 0.04; //Radius left wheel
    parameter_unicycle.radius_r = 0.04; //Radius right wheel
    parameter_unicycle.wheelbase = 0.20; //Wheelbase
    parameter_unicycle.sp_min = 0.0001; // FLT_MIN
    update_parameter_unicycle();

    vel_rif.v = 0;
    vel_rif.w = 0;
    vel_mis.v = 0;
    vel_mis.w = 0;
}

void update_parameter_unicycle(void) {
    parameter_unicycle_int.radius_l = ((int) (parameter_unicycle.radius_l * 1000.0));
    parameter_unicycle_int.radius_r = ((int) (parameter_unicycle.radius_r * 1000.0));
    parameter_unicycle_int.wheelbase = ((int) (parameter_unicycle.wheelbase * 1000.0));
    k_odo.k_left = parameter_unicycle.radius_l * parameter_motor_left.k_ang;
    k_odo.k_right = parameter_unicycle.radius_r * parameter_motor_right.k_ang;
    wheel_m = parameter_unicycle.wheelbase / 2;
    emergency.time = 1.0;
    emergency.timeout = 500;

    //Odometry
    k_odo.k_left = parameter_unicycle.radius_l * parameter_motor_left.k_ang;
    k_odo.k_right = parameter_unicycle.radius_r * parameter_motor_right.k_ang;
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
        motor_control_t motor_temp;
        motor_temp.num = -1;
        motor_temp.motor = STATE_CONTROL_VELOCITY;
        UpdateStateController(motor_temp);
    }
    /**
     * Reset time emergency
     */
    counter_stop = 0;
}

int HighLevelTaskController(void) {
    unsigned int t = TMR1; // Timing function
    int i;

    switch (control_state) {
        case STATE_CONTROL_HIGH_VELOCITY:
            /**
             * Measure linear and angular velocity for unicycle robot
             */
            VelocityMeasure();
            /**
             * Convertion linear velocity and angular velocity to motor left and motor right
             */VelToMotorReference();
            break;
        case STATE_CONTROL_HIGH_CONFIGURATION:
            break;
        default:
            for (i = 0; i < NUM_MOTORS; ++i) {
                motor_ref[i].motor = 0;
            }
            break;
    }
    return TMR1 - t; // Time of esecution
}

int deadReckoning(void) {
    unsigned int t = TMR1; // Timing function
    volatile coordinate_t delta;
    float WheelSpL = k_odo.k_left * PulsEncL; // Spostamento Ruota sinistra
    float WheelSpR = k_odo.k_right * PulsEncR; // Spostamento Ruota destra
    float SumSp = WheelSpR + WheelSpL; // Calcolo della somma degli spostamenti delle ruote
    float DifSp = WheelSpR - WheelSpL; // Calcolo della differenza degli spostamenti delle ruote
    PulsEncL = 0; // Flush variabile
    PulsEncR = 0; // Flush variabile

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

    /* TODO Verify
    if (autosend_delta_odometry) {
        // Add delta step in buffer
        delta_odometry.delta[counter_delta] = delta;
        counter_delta++;
        if (counter_delta == BUFFER_ODOMETRY) {
            abstract_message_u packet;
            packet.delta_odometry = delta_odometry;
            packet_t send = encoderSingle(createDataPacket(DELTA_ODOMETRY, HASHMAP_UNAV, &packet));
            pkg_send(HEADER_ASYNC, send);
            counter_delta = 0;
        }
    }
     */

    // Calculate odometry
    odometry(delta);

    return TMR1 - t; // Time of esecution
}

int odometry(coordinate_t delta) {
    unsigned int t = TMR1; // Timing function

    coordinate.space += delta.space;
    coordinate.x += delta.x;
    coordinate.y += delta.y;

    return TMR1 - t; // Time of esecution
}

bool Emergency(void) {
    if (save_velocity) {
        last_vel_rif.v = vel_rif.v;
        last_vel_rif.w = vel_rif.w;
        save_velocity = false;
    }
    vel_rif.v -= last_vel_rif.v * (((float) parameter_system.int_tm_mill) / 1000) / emergency.time;
    vel_rif.w -= last_vel_rif.w * (((float) parameter_system.int_tm_mill) / 1000) / emergency.time;
    if (SGN(last_vel_rif.v) * vel_rif.v < 0) vel_rif.v = 0;
    if (SGN(last_vel_rif.w) * vel_rif.w < 0) vel_rif.w = 0;
    if ((vel_rif.v == 0) && (vel_rif.w == 0)) {
        save_velocity = true;
        return true;
    }
    return false;
}

int VelToMotorReference(void) {
    unsigned int t = TMR1; // Timing function
    // >>>>> Second part: references calculation
    motor_ref[0].motor = (long int) ((1.0f / parameter_unicycle.radius_r)*(vel_rif.v + (parameter_unicycle.wheelbase * (-vel_rif.w)))*1000);
    motor_ref[1].motor = (long int) ((1.0f / parameter_unicycle.radius_l)*(vel_rif.v - (parameter_unicycle.wheelbase * (-vel_rif.w)))*1000);

    // TODO to avoid the following saturation we can normalize ref value! by Walt

    // >>>>> Saturation on 16 bit values
    motor_ref[0].motor = motor_ref[0].motor > 32767 ? 32767 : motor_ref[0].motor;
    motor_ref[0].motor = motor_ref[0].motor<-32768 ? -32768 : motor_ref[0].motor;

    motor_ref[1].motor = motor_ref[1].motor > 32767 ? 32767 : motor_ref[1].motor;
    motor_ref[1].motor = motor_ref[1].motor<-32768 ? -32768 : motor_ref[1].motor;
    // <<<<< Saturation on 16 bit values

    return TMR1 - t; // Time of esecution
}

int VelocityMeasure(void) {
    unsigned int t = TMR1; // Timing function

    long vel_v = (parameter_unicycle_int.radius_r * motor_right.measure_vel + parameter_unicycle_int.radius_l * motor_left.measure_vel) / 2;
    long vel_w = (parameter_unicycle_int.radius_r * motor_right.measure_vel - parameter_unicycle_int.radius_l * motor_left.measure_vel) / (2 * parameter_unicycle_int.wheelbase);
    vel_mis.v = ((float) vel_v / 1000000);
    vel_mis.w = ((float) vel_w / 1000);

    return TMR1 - t; // Time of esecution
}