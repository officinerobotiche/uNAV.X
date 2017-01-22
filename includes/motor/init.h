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

#ifndef MOTOR_INIT_H
#define	MOTOR_INIT_H

#ifdef	__cplusplus
extern "C" {
#endif

/******************************************************************************/
/* System Level #define Macros                                                */
/******************************************************************************/

#include "motor/motor.h"
    
/**
 * Numbers of motors available in this board
 */
#define MOTOR_SIZE 2
#define MOTOR_ZERO 0
#define MOTOR_ONE  1
    
/******************************************************************************/
/* System Function Prototypes                                                 */
/******************************************************************************/
/**
 * Initialization all motor
 */
void Motor_start(void);
    
#ifdef	__cplusplus
}
#endif

#endif	/* MOTOR_INIT_H */

