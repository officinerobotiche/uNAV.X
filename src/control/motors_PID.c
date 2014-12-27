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
#include <pwm12.h>
#include "control/motors_PID.h"       /* variables/params used by motorsPID.c         */
#include "system/system.h"
#include "system/user.h"
#include "packet/packet.h"

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

tPID PIDstruct1; // PID motore Sinistra
fractional abcCoefficient1[3] __attribute__((section(".xbss, bss, xmemory")));
fractional controlHistory1[3] __attribute__((section(".ybss, bss, ymemory")));
fractional kCoeffs1[3]; //Coefficienti KP, KI, KD Per PID1 Sinistra
tPID PIDstruct2; //PID motore Destra
fractional abcCoefficient2[3] __attribute__((section(".xbss, bss, xmemory")));
fractional controlHistory2[3] __attribute__((section(".ybss, bss, ymemory")));
fractional kCoeffs2[3]; //Coefficienti KP, KI, KD Per PID2 Destra

// ADC buffer, 2 channels (AN0, AN1), 32 bytes each, 2 x 32 = 64 bytes
int AdcBuffer[2][ADC_BUFF] __attribute__((space(dma), aligned(256)));

/** */
//From System
extern parameter_system_t parameter_system;

volatile int PulsEncL = 0; //Buffer for deadReckoning
volatile int PulsEncR = 0; //Buffer for deadReckoning

parameter_motors_t parameter_motors;
constraint_t constraint;
velocity_t vel_rif, vel_mis;
pid_control_t pid_left, pid_right;
enable_motor_t enable_motors;
motor_t motor_left, motor_right;

//variables for emergency
emergency_t emergency;
velocity_t last_vel_rif;
bool save_velocity = true;

float const_vel[3];

typedef struct parameter_int {
    long radius_l;
    long radius_r;
    long wheelbase;
    long k_vel_l;
    long k_vel_r;
} parameter_int_t;

k_odo_t k_odo;
float wheel_m;

parameter_int_t parameter_int;

/**/

extern volatile unsigned long timePeriodL; //Periodo Ruota Sinistra
extern volatile unsigned long timePeriodR; //Periodo Ruota Destra
extern volatile unsigned SIG_VELL; //Verso rotazione ruota Sinistra
extern volatile unsigned SIG_VELR; //Verso rotazione ruota Destra

/******************************************************************************/
/* User Functions                                                             */

/******************************************************************************/

void init_parameter(void) {
    parameter_motors.radius_l = 0.04; //Raggio ruota sinistra
    parameter_motors.radius_r = 0.04; //Raggio ruota destra
    parameter_motors.wheelbase = 0.20; //Interasse
    parameter_motors.k_vel_l = K_VEL; //Costante velocità
    parameter_motors.k_vel_r = K_VEL;
    parameter_motors.k_ang_l = K_ANG; //Constante angolare
    parameter_motors.k_ang_r = K_ANG;
    parameter_motors.sp_min = 0.0001; // FLT_MIN
    parameter_motors.pwm_step = 4096;

    update_parameter();

    vel_rif.v = 0;
    vel_rif.w = 0;
    vel_mis.v = 0;
    vel_mis.w = 0;
    motor_left.control_vel = 0;
    motor_left.measure_vel = 0;
    motor_left.refer_vel = 0;
    motor_left.current = 0;
    motor_right.control_vel = 0;
    motor_right.measure_vel = 0;
    motor_right.refer_vel = 0;
    motor_right.current = 0;

    constraint.max_left = 14000;
    constraint.max_right = 14000;

    const_vel[0] = 1;

    enable_motors = false;
}

void update_parameter(void) {
    parameter_int.radius_l = ((int) (parameter_motors.radius_l * 1000.0));
    parameter_int.radius_r = ((int) (parameter_motors.radius_r * 1000.0));
    parameter_int.wheelbase = ((int) (parameter_motors.wheelbase * 1000.0));
    parameter_int.k_vel_r = parameter_motors.k_vel_r;
    parameter_int.k_vel_l = parameter_motors.k_vel_l;
    k_odo.k_left = parameter_motors.radius_l * parameter_motors.k_ang_l;
    k_odo.k_right = parameter_motors.radius_r * parameter_motors.k_ang_r;
    wheel_m = parameter_motors.wheelbase / 2;
    emergency.time = 1.0;
    emergency.timeout = 500;
}

void init_pid_control(void) {
    pid_left.kp = 0.6;
    pid_left.ki = 0.7;
    pid_left.kd = 0.1;
    pid_right.kp = 0.6;
    pid_right.ki = 0.7;
    pid_right.kd = 0.1;
}

void update_pid_l(void) {
    kCoeffs1[0] = Q15(pid_left.kp); //0.5
    kCoeffs1[1] = Q15(pid_left.ki); //0.6
    kCoeffs1[2] = Q15(pid_left.kp); //0.0
    InitPid1(); //Init PIDL
}

void update_pid_r(void) {
    kCoeffs2[0] = Q15(pid_right.kp);
    kCoeffs2[1] = Q15(pid_right.ki);
    kCoeffs2[2] = Q15(pid_right.kd);
    InitPid2(); //Init PIDR
}

void InitPid1(void) {
    //Initialize the PID data structure: PIDstruct
    //Set up pointer to derived coefficients
    PIDstruct1.abcCoefficients = &abcCoefficient1[0];
    //Set up pointer to controller history samples
    PIDstruct1.controlHistory = &controlHistory1[0];
    // Clear the controler history and the controller output
    PIDInit(&PIDstruct1);
    //Derive the a,b, & c coefficients from the Kp, Ki & Kd
    PIDCoeffCalc(&kCoeffs1[0], &PIDstruct1);
}

void InitPid2(void) {
    //Initialize the PID data structure: PIDstruct
    //Set up pointer to derived coefficients
    PIDstruct2.abcCoefficients = &abcCoefficient2[0];
    //Set up pointer to controller history samples
    PIDstruct2.controlHistory = &controlHistory2[0];
    // Clear the controler history and the controller output
    PIDInit(&PIDstruct2);
    //Derive the a,b, & c coefficients from the Kp, Ki & Kd
    PIDCoeffCalc(&kCoeffs2[0], &PIDstruct2);
}

bool Emergency(void) {
    if (save_velocity) {
        last_vel_rif.v = vel_rif.v;
        last_vel_rif.w = vel_rif.w;
        save_velocity = false;
    }
    vel_rif.v -= last_vel_rif.v * (((float)parameter_system.int_tm_mill) / 1000) / emergency.time;
    vel_rif.w -= last_vel_rif.w * (((float)parameter_system.int_tm_mill) / 1000) / emergency.time;
    if (SGN(last_vel_rif.v) * vel_rif.v < 0) vel_rif.v = 0;
    if (SGN(last_vel_rif.w) * vel_rif.w < 0) vel_rif.w = 0;
    if((vel_rif.v == 0) && (vel_rif.w == 0)) {
        save_velocity = true;
        return true;
    }
    return false;
}

int Velocity(void) {
    unsigned int t = TMR1; // Timing function
    int rifer_con_left, rifer_con_right;
    MOTOR_ENABLE1 = enable_motors;
    MOTOR_ENABLE2 = enable_motors;
    long vel_v = (parameter_int.radius_r * motor_right.measure_vel + parameter_int.radius_l * motor_left.measure_vel) / 2;
    long vel_w = (parameter_int.radius_r * motor_right.measure_vel - parameter_int.radius_l * motor_left.measure_vel) / (2 * parameter_int.wheelbase);
    vel_mis.v = ((float) vel_v / 1000000);
    vel_mis.w = ((float) vel_w / 1000);

    rifer_con_left = (int) ((1.0f / parameter_motors.radius_r)*(vel_rif.v + (parameter_motors.wheelbase * (-vel_rif.w)))*1000);
    rifer_con_right = (int) ((1.0f / parameter_motors.radius_l)*(vel_rif.v - (parameter_motors.wheelbase * (-vel_rif.w)))*1000);
    // Calculating constraint
    if (abs(rifer_con_left) > constraint.max_left) {
        motor_left.refer_vel = SGN(rifer_con_left) * constraint.max_left;
    } else {
        motor_left.refer_vel = rifer_con_left;
    }
    if (abs(rifer_con_right) > constraint.max_right) {
        motor_right.refer_vel = SGN(rifer_con_right) * constraint.max_right;
    } else {
        motor_right.refer_vel = rifer_con_right;
    }
    return TMR1 - t; // Time of esecution
}

int MotorPIDL(void) {
    unsigned int t = TMR1; //Timing funzione
    unsigned long timePeriodLtmp; //Variabili temporanee Periodo
    int SIG_VELLtmp; //Segno velocità

    timePeriodLtmp = timePeriodL; //Salvataggio TimerPeriod
    timePeriodL = 0; //Pulizia variabile
    SIG_VELLtmp = SIG_VELL; //Salvataggio Segno velocità
    SIG_VELL = 0; //Pulizia variabile
    motor_left.measure_vel = 0; //Flush variaibile velocità R

    PulsEncL += (int) POS1CNT; //Salvataggio spazio percorso
    POS1CNT = 0; //Reset registro
    //calcolo della velocità
    //Verifica SIG_VELLtmp!=0 & calcolo velocità
    if (SIG_VELLtmp) motor_left.measure_vel = SIG_VELLtmp * (parameter_motors.k_vel_l / timePeriodLtmp);
    PIDstruct1.controlReference = motor_left.refer_vel; //Riferimento Ruota Sinistra
    PIDstruct1.measuredOutput = motor_left.measure_vel; //Misura velocità
    PID(&PIDstruct1); //Esecuzione funzione PID
    motor_left.control_vel = -(PIDstruct1.controlOutput >> 4) + 2049; //Conversione valore per PWM
    //Invio dell'azione di controllo al motore per mezzo del PWM
    SetDCMCPWM1(1, motor_left.control_vel, 0);

    return TMR1 - t; //Misura tempo di esecuzione
}

int MotorPIDR(void) {
    unsigned int t = TMR1; //Timing funzione
    unsigned long timePeriodRtmp; //Variabili temporanee Periodo
    int SIG_VELRtmp; //Segno velocità

    timePeriodRtmp = timePeriodR; //Salvataggio TimerPeriod
    timePeriodR = 0; //Pulizia variabile
    SIG_VELRtmp = SIG_VELR; //Salvataggio Segno velocità
    SIG_VELR = 0; //Pulizia variabile
    motor_right.measure_vel = 0; //Flush variabile velocità R

    PulsEncR += (int) POS2CNT; //Salvataggio spazio percorso
    POS2CNT = 0; //Reset registro
    //calcolo della velocità
    //Verifica SIG_VELLtmp!=0 & calcolo velocità
    if (SIG_VELRtmp) motor_right.measure_vel = SIG_VELRtmp * (parameter_motors.k_vel_r / timePeriodRtmp);
    PIDstruct2.controlReference = motor_right.refer_vel; //Riferimento Ruota Destra
    PIDstruct2.measuredOutput = motor_right.measure_vel; //Misura velocità
    PID(&PIDstruct2); //Esecuzione funzione PID
    motor_right.control_vel = (PIDstruct2.controlOutput >> 4) + 2049; //Conversione valore per PWM
    //Invio dell'azione di controllo al motore per mezzo del PWM
    SetDCMCPWM1(2, motor_right.control_vel, 0);

    return TMR1 - t; //Misura tempo di esecuzione
}

void adc_motors_current(void) {
    int SIG_VELLtmp, SIG_VELRtmp; //Segno velocità
    int AdcCount = 0; //Contatore
    long ADCValueTmp[ADC_CHANNELS] = {0, 0}; //Valore temporaneo letture

    for (AdcCount = 0; AdcCount < ADC_BUFF; AdcCount++) // Calcolo media di tutti i campioni
    {
        ADCValueTmp[0] += AdcBuffer[0][AdcCount]; //Sommatoria per AN0
        ADCValueTmp[1] += AdcBuffer[1][AdcCount]; //Sommatoria per AN1
    }
    SIG_VELLtmp = SIG_VELL; //Salvataggio Segno velocità
    SIG_VELRtmp = SIG_VELR; //Salvataggio Segno velocità
    motor_left.current = SIG_VELRtmp * ADCValueTmp[0] >> 6; //Divisione o shifth
    motor_right.current = SIG_VELRtmp * ADCValueTmp[1] >> 6; //Divisione o shifth
}
