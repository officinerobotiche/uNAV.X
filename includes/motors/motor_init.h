/*
 * Copyright (C) 2015 Officine Robotiche
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
    
    /**
     * Numbers of motors available in this board
     */
#define NUM_MOTORS 2
#define MOTOR_ZERO 0
#define MOTOR_ONE 1
    
#include "motors/motor.h"

    /** 
     * Initialization QEI motors
     * @param motIdx number motor
     */
    void InitQEI(MOTOR_t *motor);
    /** 
     * Initialization Input Capture motors
     * @param motIdx number motor
     */
    void InitIC(MOTOR_t *motor);
    /** 
     * Initialization Timer 2 for IC (Input Capture)
     */
    void InitTimer2(void);

    /**
     * Initialization motor
     */
    void Motor_Init();
    
    /**
     * Select the correct Input Capture prescaler		
     * @param motIdx number motor				
     */		
    void SelectIcPrescaler(MOTOR_t *motor);
    
    /** 
     * Safely switch to the new Input Capture prescaler
     * @param motIdx number motor
     * @param mode
     */
    inline void SwitchIcPrescaler(int motIdx, int mode);


#ifdef	__cplusplus
}
#endif

#endif	/* MOTOR_INIT_H */

