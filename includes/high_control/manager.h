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

#include "motors/motor_control.h"

    //Numbers and names associated at all processes
#define PROCESS_MOTION_LENGTH 2
#define PROCESS_VELOCITY 0
#define VELOCITY_STRING "Velocity"
#define PROCESS_ODOMETRY 1
#define ODOMETRY_STRING "Odometry"
    
#define MOTOR_LEFT MOTOR_ZERO
#define MOTOR_RIGHT MOTOR_ONE
    
    typedef void (*control_task_init_t) (motor_state_t*);
    typedef diff_drive_velocity_t (*control_task_loop_t) (diff_drive_velocity_t*, diff_drive_coordinate_t*);

    /*************************************************************************/
    /* System Function Prototypes                                            */
    /*************************************************************************/

    bool add_task(bool autostart, control_task_init_t init, control_task_loop_t loop);
    
    void HighControl_Init(void);
    
    void reset_motion(void);
    /**
     * Initialization all parameters for motor controller.
     */
    diff_drive_parameter_unicycle_t init_motion_parameter_unicycle(void);
    /**
     * Get all parameters for motor controller.
     */
    inline diff_drive_parameter_unicycle_t get_motion_parameter_unicycle(void);
    /**
     * Function to update parameters relative a parameter message
     */
    void update_motion_parameter_unicycle(diff_drive_parameter_unicycle_t parameter_unicycle);

    /**
     * Initialization coordinate for odometry
     */
    diff_drive_coordinate_t init_motion_coordinate(void);
    /**
     * 
     * @return 
     */
    inline diff_drive_coordinate_t get_motion_coordinate(void);
    /**
     * Update stored sine and cosine with new theta value
     */
    void update_motion_coordinate(diff_drive_coordinate_t coordinate);

    /**
     * Start deadReckoning operation
     * @return time to compute this function
     */
    int deadReckoning(void);
    
    /**
     * Get motion state
     * @return motion state
     */
    inline diff_drive_state_t get_motion_state(void);

    /**
     * Update state controller for high level control
     * @param state set state to high level motion control
     */
    void set_motion_state(diff_drive_state_t state);

    /**
     *
     * @return time to compute this function
     */
    void HighLevelTaskController(int argc, int *argv);

    /**
     * Evaluate linear and angular velocity from unicycle robot.
     * @param set velocity to control
     * @return time to compute this function
     */
    void set_motion_velocity_ref_unicycle(diff_drive_velocity_t vel_rif);

    /**
     * Return velocity unicycle robot
     * @return return velocity unicycle
     */
    inline diff_drive_velocity_t get_motion_velocity_ref_unicycle(void);

    /**
     * 
     * @return 
     */
    inline diff_drive_velocity_t get_motion_velocity_meas_unicycle(void);
    /**
     * Conversion data from rotor motors measure and save value for velocity.
     * @return time to compute this function
     */
    int VelocityMeasure(void);

#ifdef	__cplusplus
}
#endif

#endif	/* MANAGER_H */

