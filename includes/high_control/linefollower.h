/*
 * Copyright (C) 2015 Officine Robotiche
 * Author: Mauro Soligo
 * email:  mauro.soligo@officinerobotiche.it
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


#ifndef LINEFOLLOWER_H
#define	LINEFOLLOWER_H


#ifdef	__cplusplus
extern "C" {
#endif

    /**************************************************************************/
    /* System Function Prototypes                                             */
    /**************************************************************************/
    
    void init_linefollower (motor_state_t* state);

    motion_velocity_t loop_linefollower (motion_velocity_t* measure, motion_coordinate_t* coordinate);
    
    /**************************************************************************/
    /* IR sensor reading related functions                                    */
    /**************************************************************************/    
    void IRsensor(void);
    
#ifdef	__cplusplus
}
#endif

#endif	/* LINEFOLLOWER_H */
