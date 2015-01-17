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

#ifndef MOTORSPID_H
#define	MOTORSPID_H

#ifdef	__cplusplus
extern "C" {
#endif

    /******************************************************************************/
    /* System Level #define Macros                                                */
    /******************************************************************************/

    //Start define with fixed K_vel convertion velocity
#define K_VEL 27925268.03190926
    //Start define with fixed K_ang convertion angular
#define K_ANG 0.000174532925199

    //Internal definition gain for odometry

    typedef struct k_odo {
        float k_left;
        float k_right;
    } k_odo_t;

    /**
     * Define to select current state of control
     */
#define DIRECT_CONTROL_STATE 0
#define POSITION_CONTROL_STATE 1
#define VELOCITY_CONTROL_STATE 2
#define TORQUE_CONTROL_STATE 3

#define DISABLE_HIGH_CONTROL_STATE 0
#define VELOCITY_UNICYCLE_CONTROL_STATE 1
#define CONFIGURATION_CONTROL_STATE 2

    /******************************************************************************/
    /* System Function Prototypes                                                 */
    /******************************************************************************/

    /**
     * Initialization all parameters for motor controller.
     */
    void init_parameter(void);

    /**
     * Function to update motor parameters from message
     */
    void update_parameter_motors(void);

    /**
     * Initializatin standard value for PID controllers
     */
    void init_pid_control(void);

    /**
     * Transform float value recived from gain for PID left in Q15 value
     * for dsp controller.
     */
    void update_pid_l(void);

    /**
     * Transform float value recived from gain for PID right in Q15 value
     * for dsp controller.
     */
    void update_pid_r(void);

    /**
     * PID data structure: PIDstruct for PID 1 (Motor left)
     */
    void InitPid1(void);

    /**
     * PID data structure: PIDstruct for PID 1 (Motor right)
     */
    void InitPid2(void);

    /**
     * Evaluate linear and angular velocity from unicycle robot. Convertion data
     * from rotor motors measure and save value for velocity.
     * @return time to compute parsing packet
     */

    /**
     * Control type of law control in action and select, convert and check
     * reference.
     * @return Time to Compute task control reference
     */
    int MotorTaskController(void);

    /**
     * Esecution velocity PID for left motor
     *           _____          _______
     * ref +    |     |  cont  |       |
     * --->o--->| PID |------->| Motor | -|-> measure
     *   -/|\   |_____|        |_______|  |
     *     |______________________________|
     * We have three step for esecution PID controller on motor:
     * 1. Evalute measure of velocity rotor (dtheta) are combined information
     * from Input Capture elaboration from relative encoder and QEI module
     * direction of rotation. In same time is saved pulse from QEI module. (This
     * information is important for odometry)
     * 2. Load data (reference, measure) and esecution PID control and get value
     * 3. Convertion PID value for PWM controller
     * @return time to compute parsing packet
     */
    int MotorPIDL(void);

    /**
     * Esecution velocity PID for right motor
     *           _____          _______
     * ref +    |     |  cont  |       |
     * --->o--->| PID |------->| Motor | -|-> measure
     *   -/|\   |_____|        |_______|  |
     *     |______________________________|
     * We have three step for esecution PID controller on motor:
     * 1. Evalute measure of velocity rotor (dtheta) are combined information
     * from Input Capture elaboration from relative encoder and QEI module
     * direction of rotation. In same time is saved pulse from QEI module. (This
     * information is important for odometry)
     * 2. Load data (reference, measure) and esecution PID control and get value
     * 3. Convertion PID value for PWM controller
     * @return time to compute parsing packet
     */
    int MotorPIDR(void);

    /**
     * Mean valure for current measure motors
     */
    void adc_motors_current(void);

#ifdef	__cplusplus
}
#endif

#endif	/* MOTORSPID_H */