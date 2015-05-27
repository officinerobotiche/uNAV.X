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

#ifndef CARTESIAN_H
#define	CARTESIAN_H

#ifdef	__cplusplus
extern "C" {
#endif

void init_cartesian (motor_state_t* state);

motion_velocity_t loop_cartesian (motion_velocity_t* measure, motion_coordinate_t* coordinate);


#ifdef	__cplusplus
}
#endif

#endif	/* CARTESIAN_H */

