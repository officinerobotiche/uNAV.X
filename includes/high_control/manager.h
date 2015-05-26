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

#ifndef MANAGER_H
#define	MANAGER_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "packet/packet.h"

    //Numbers and names associated at all processes
#define PROCESS_MOTION_LENGTH 2
#define PROCESS_VELOCITY 0
#define VELOCITY_STRING "Velocity"
#define PROCESS_ODOMETRY 1
#define ODOMETRY_STRING "Odometry"

    /*************************************************************************/
    /* System Function Prototypes                                            */
    /*************************************************************************/

    /**
     * Initialization all parameters for motor controller.
     */
    void init_parameter_unicycle(void);

    /**
     * Function to update parameters relative a parameter message
     */
    void update_parameter_unicycle(void);

    /**
     * Initalization coordinate for odometry
     */
    void init_coordinate(void);

    /**
     * Update stored sine and cosine with new theta value
     */
    void update_coord(void);

    /**
     * Start deadReckoning operation
     * @return time to compute this function
     */
    int deadReckoning(void);

    /**
     *
     * @param delta
     * @return time to compute this function
     */
    int odometry(coordinate_t delta);

    /**
     * Update state controller for high level control
     */
    void UpdateHighStateController(int state);

    /**
     *
     * @return time to compute this function
     */
    int HighLevelTaskController(void);

    /**
     * Evaluate linear and angular velocity from unicycle robot.
     * @param set velocity to control
     * @return time to compute this function
     */
    int set_high_velocity(velocity_t vel_rif);

    /**
     * Return velocity unicycle robot
     * @return return velocity unicycle
     */
    inline velocity_t get_high_velocity_ref(void);

    /**
     * Conversion data from rotor motors measure and save value for velocity.
     * @return time to compute this function
     */
    int VelocityMeasure(void);

#ifdef	__cplusplus
}
#endif

#endif	/* MANAGER_H */

