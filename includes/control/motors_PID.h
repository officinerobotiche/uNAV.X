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
     * Numbers of motors available in this board
     */
#define NUM_MOTORS 2
#define REF_MOTOR_LEFT 0
#define REF_MOTOR_RIGHT 1

#define ENC_BEFORE_GEAR 1
#define ENC_AFTER_GEAR 0
#define VOLT_BRIDGE 12
#define CPR 300
#define RATIO 30
    // If CPR is before ratio
    //    ThC = CPR * RATIO   
    // else
    //    ThC = RATIO
    //Start define with fixed K_vel conversion velocity
    // KVEL = FRTMR2 *  [ 2*pi / ( ThC * 2 ) ] * 1000 (velocity in milliradiant)
#define K_VEL 27925268.03190926
    //Start define with fixed K_ang conversion angular
    // K_ANG = 2*PI / ( ThC * (QUADRATURE = 4) )
#define K_ANG 0.000174532925199

#define DEFAULT_KP 0.6
#define DEFAULT_KI 0.15
#define DEFAULT_KD 0.2
    
    typedef struct _ICdata {
        volatile unsigned int overTmr; //Overflow timer
        volatile unsigned long timePeriod; //Time period from Input Capture
        volatile int SIG_VEL; //Sign of versus rotation motor
    } ICdata;
    
    /******************************************************************************/
    /* System Function Prototypes                                                 */
    /******************************************************************************/

    /**
     * Initialization all variables for motor controller.
     * @param num Number motor
     */
    void init_motor(short num);
    /**
     * Initialization parameters for motor controller.
     * @param num Number motor
     * @return Default configuration
     */
    parameter_motor_t init_parameter_motors(short num);
    /**
     * Function to update motor parameters from message
     * @param num Number motor
     */
    void update_parameter_motors(short num, parameter_motor_t parameter);
    /**
     * Function to update motor constraints from message
     * @param num Number motor
     * @param constraint constraints set
     */
    void update_constraints_motor(short num, motor_t constraint);
    /**
     * Initialization standard value for PID controllers
     * @param number Number motor
     * @return Default configuration
     */
    pid_control_t init_pid_control(short num);
    /**
     * Transform float value received from gain for PID right in Q15 value
     * for dsp controller.
     * PID data structure: PIDstruct for PID 1 (Motor left)
     * PID data structure: PIDstruct for PID 1 (Motor right)
     * @param num Number motor
     */
    void update_pid(short num, pid_control_t pid);
    /**
     * Return value of PID controller
     * @param motIdx number of motor
     * @return value PID
     */
    inline pid_control_t get_pid_value(short motIdx);
    /**
     * Initialization standard value for emergency configuration motor
     * @param number Number motor
     * @return default configuration for emergency stop
     */
    emergency_t init_parameter_emergency(short num);
    /**
     * Initialization standard value for constraints motor
     * @param number Number motor
     * @return default configuration for constraints
     */
    motor_t init_parameter_constraints(short num);
    /**
     * Update counter and step value for emergency controller.
     * @param number Number motor
     * @param emergency configuration to save
     */
    void update_parameter_emergency(short num, emergency_t emergency_data);
    /**
     * Write a correct value of motor reference and if necessary modify
     * reference to control constraint.
     * @param motor Number motor
     * @param ref_velocity reference of velocity
     * @return Time to compute this function
     */
    int set_motor_velocity(short motor, int16_t ref_velocity);
    /**
     * Return information about state motor, torque velocity position.
     * @param motIdx number of motor
     * @return return information about motor
     */
    inline motor_t get_motor_measure(short motIdx);
    /**
     * Return information about motor reference of control.
     * @param motIdx number of motor
     * @return return information about motor
     */
    inline motor_t get_motor_reference(short motIdx);
    /**
     * Return information about motor constraints.
     * @param motIdx number of motor
     * @return return information about motor
     */
    inline motor_t get_motor_constraints(short motIdx);
    /**
     * Set state controller for all motors, if DISABLE, set enable motor to zero
     * @param num number motor to update state if -1 set all motor to state
     * @param motor state command
     */
    void UpdateStateController(short num, motor_control_t motor);
    /**
     * Return parameters from motor
     * @param motIdx number selected motor
     * @return parameter motors
     */
    inline parameter_motor_t get_parameter_motor(short motIdx);
    /**
     * If not receive anything velocity messages. Start controlled stop motors
     * @param number Number motor
     * @return start emergency mode or not.
     */
    bool Emergency(short num);
    
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
     * Return state of motor
     * @param motIdx number of motor selected
     * @return state of motor
     */
    inline state_controller_t get_motor_state(short motIdx);

    /**
     * Measure velocity from Input Capture and QEI
     * @param number Number motor
     * @return Time to Compute task control reference
     */
    int measureVelocity(short num);

    /**
     * Execution velocity PID for left motor
     *           _____          _______
     * ref +    |     |  cont  |       |
     * --->o--->| PID |------->| Motor | -|-> measure
     *   -/|\   |_____|        |_______|  |
     *     |______________________________|
     * We have three step for execution PID controller on motor:
     * 1. Evaluate measure of velocity rotor (dtheta) are combined information
     * from Input Capture elaboration from relative encoder and QEI module
     * direction of rotation. In same time is saved pulse from QEI module. (This
     * information is important for odometry)
     * 2. Load data (reference, measure) and execution PID control and get value
     * 3. Conversion PID value for PWM controller
     * @param number Number motor
     * @return time to compute parsing packet
     */
    int MotorPID(short num);

    /**
     * Mean value for current measure motors
     */
    void adc_motors_current(void);

#ifdef	__cplusplus
}
#endif

#endif	/* MOTORSPID_H */