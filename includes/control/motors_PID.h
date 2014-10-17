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
/* System Level #define Macros                                                */
/******************************************************************************/

#define K_VEL 27925268.03190926
#define K_ANG 0.000174532925199

/******************************************************************************/
/* System Function Prototypes                                                 */
/******************************************************************************/

typedef struct k_odo {
    float k_left;
    float k_right;
} k_odo_t;

void init_parameter(void);
void update_parameter(void);
void init_pid_control(void);
void update_pid_l(void);
void update_pid_r(void);
void InitPid1(void); /* PID data structure: PIDstruct for PID 1 (Motor left) */
void InitPid2(void); /* PID data structure: PIDstruct for PID 2 (Motor right) */
bool Emergency(void);
int Velocity(void);
int MotorPIDL(void); /* Esecution velocity PID for left motor */
int MotorPIDR(void); /* Esecution velocity PID for right motor */
void adc_motors_current(void); /* Mean valure for current measure motors */