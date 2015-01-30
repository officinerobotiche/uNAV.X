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

    /**************************************************************************/
    /* System Level #define Macros                                            */
    /**************************************************************************/

    /**
     * Numbers of motors avaiable in this board
     */
#define NUM_MOTORS 2
    //Start define with fixed K_vel convertion velocity
#define K_VEL 27925268.03190926
    //Start define with fixed K_ang convertion angular
#define K_ANG 0.000174532925199

#define DEFAULT_KP 0.6
#define DEFAULT_KI 0.15
#define DEFAULT_KD 0.2

    // >>>>> Speed zones in millirad/sec
#define MIN1 1600  // about 15 RPM
#define MAX1 2600  // about 25 RPM
#define MIN2 5500  // about 50 RPM
#define MAX2 6500  // about 60 RPM
#define MIN3 20000 // about 190 RPM
#define MAX3 22000 // about 210 RPM
    // <<<<< Speed zones in millirad/sec

    //Internal definition gain for odometry

    typedef struct k_odo {
        float k_left;
        float k_right;
    } k_odo_t;

    /******************************************************************************/
    /* System Function Prototypes                                                 */
    /******************************************************************************/

    /**
     * Initialization all parameters for motor controller.
     */
    void init_parameter_motors(void);

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
     * Write a correct value of motor reference and if necessary modify
     * reference to control contraint.
     * @param number Number motor
     * @return Time to compute this function
     */
    int MotorVelocityReference(short motor);

    /**
     * Set state controller for all motors, if DISABLE, set enable motor to zero
     * @param num number motor to update state if -1 set all motor to state
     * @param motor state command
     */
    void UpdateStateController(short num, motor_control_t motor);

    /**
     * Convert and check reference for type of law control selected. We have
     * four principal type of control motor:
     *  - Direct control (write direct PWM)
     *  - Position control (move to desired angle)
     *  - Velocity control (move to desired angular velocity)
     *  - Torque control (move to desired torque)
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
     * Select the correct Input Capture prescaler
     */
    void SelectIcPrescaler(int motIdx);

    /**
     * Mean valure for current measure motors
     */
    void adc_motors_current(void);

#ifdef	__cplusplus
}
#endif

#endif	/* MOTORSPID_H */