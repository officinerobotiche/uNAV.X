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

#ifndef MOTOR_CONTROL_H
#define	MOTOR_CONTROL_H

#ifdef	__cplusplus
extern "C" {
#endif

    #include <stdint.h>
    #include <stdbool.h>
    #include <string.h>
    #include <dsp.h>

    #include <system/task_manager.h>
    #include <peripherals/gpio.h>

    #include "system/peripherals.h"
    
    /**************************************************************************/
    /* System Level #define Macros                                            */
    /**************************************************************************/

    /**
     * Numbers of motors available in this board
     */
#define NUM_MOTORS 2
#define MOTOR_ZERO 0
#define MOTOR_ONE 1

//Numbers and names associated at all processes
#define PROCESS_MOTOR_LENGTH 4
#define LEFT_PROCESS_PID 0
#define LEFT_PROCESS_PID_STRING "Left/PID"
#define RIGHT_PROCESS_PID 1
#define RIGHT_PROCESS_PID_STRING "Right/PID"
#define LEFT_PROCESS_MEASURE 3
#define LEFT_PROCESS_MEASURE_STRING "Left/Meas"
#define RIGHT_PROCESS_MEASURE 4
#define RIGHT_PROCESS_MEASURE_STRING "Right/Meas"
    
    typedef enum {
        CONTROL_EMERGENCY = -1,     ///< Motors slow down to zero speed, then the bridge is turned off
        CONTROL_DISABLE = 0,        ///< Motors disabled
        CONTROL_POSITION = 1,       ///< Motors controlled in position
        CONTROL_VELOCITY = 2,       ///< Motors controlled in velocity
        CONTROL_TORQUE = 3,         ///< Motors controller in torque
        CONTROL_DIRECT = 4,         ///< Motors controlled using direct PWM signals
    } enum_state_t;
    
    typedef struct _ICdata {
        volatile unsigned int delta;
        volatile unsigned int k_mul;        // k_vel multiplier according to IC scale
        volatile unsigned int overTmr;      //Overflow timer
        volatile unsigned int oldTime;      //Old time stored
        volatile unsigned long timePeriod;  //Time period from Input Capture
        volatile int SIG_VEL;               //Sign of versus rotation motor
        volatile unsigned short number;       //Mode of Input Capture
    } ICdata;
    
    typedef void (*event_prescaler_t)(int motIdx);

    /******************************************************************************/
    /* System Function Prototypes                                                 */
    /******************************************************************************/

    /**
     * Initialization all variables for motor controller.
     * @param motIdx Number motor
     * @param enable_ GPIO for enable
     * @param ICinfo_ Input capture information
     * @param current_ Analog pin number for current
     * @param voltage_ Analog pin number for temperature
     * @return task to run the MotorControlManager
     */
    hTask_t init_motor(const short motIdx, gpio_t* enable_, ICdata* ICinfo_, event_prescaler_t prescaler_event, int current_, int voltage_);
    
    /**
     * Initialization parameters for motor controller.
     * @return Default configuration
     */
    motor_parameter_t init_motor_parameters();
    /**
     * Return parameters from motor
     * @param motIdx number selected motor
     * @return parameters motor
     */
    inline motor_parameter_t get_motor_parameters(short motIdx);
    /**
     * Function to update motor parameters from message
     * @param motIdx Number motor
     */
    void update_motor_parameters(short motIdx, motor_parameter_t parameters);
    
    /**
     * Initialization standard value for constraints motor
     * @return default configuration for constraints
     */
    motor_t init_motor_constraints();
    /**
     * Return information about motor constraints.
     * @param motIdx number of motor
     * @return return information about motor
     */
    inline motor_t get_motor_constraints(short motIdx);
    /**
     * Function to update motor constraints from message
     * @param motIdx Number motor
     * @param constraint constraints set
     */
    void update_motor_constraints(short motIdx, motor_t constraints);
    
    /**
     * Initialization standard value for PID controllers
     * @return Default configuration
     */
    motor_pid_t init_motor_pid();
    /**
     * Return value of PID controller
     * @param motIdx number of motor
     * @param type type of controller (position, velocity, current)
     * @return value PID
     */
    inline motor_pid_t get_motor_pid(short motIdx, enum_state_t type);
    /**
     * Transform float value received from gain for PID right in Q15 value
     * for dsp controller.
     * @param motIdx Number motor
     * @param type type of controller (position, velocity, current)
     * @param pid update data for PID controller 
     * @return return true if the PID gains are correctly stored
     */
    bool update_motor_pid(short motIdx, enum_state_t type, motor_pid_t pid);

    /**
     * Initialization standard value for emergency configuration motor
     * @return default configuration for emergency stop
     */
    motor_emergency_t init_motor_emergency();
    /**
     * Return emergency parameters from motor
     * @param motIdx number selected motor
     * @return emergency parameters motor
     */
    inline motor_emergency_t get_motor_emergency(short motIdx);
    /**
     * Update counter and step value for emergency controller.
     * @param motIdx Number motor
     * @param emergency configuration to save
     */
    void update_motor_emergency(short motIdx, motor_emergency_t emergency);
    
    /**
     * Return information about state motor, torque velocity position.
     * @param motIdx number of motor
     * @return return information about motor
     */
    inline motor_t get_motor_measures(short motIdx);
    /**
     * Return information about diagnostic, current, temperature, etc etc.
     * @param motIdx number of motor
     * @return return diagnostic information about motor
     */
    inline motor_diagnostic_t get_motor_diagnostic(short motIdx);
    /**
     * Return information about motor reference of control.
     * @param motIdx number of motor
     * @return return information about motor
     */
    inline motor_t get_motor_reference(short motIdx);
    /**
     * Set position motor.
     * @param motIdx number of motor
     * @param value new value position
     */
    inline void reset_motor_position_measure(short motIdx, motor_control_t value);
    /**
     * Write a correct value of motor reference and if necessary modify
     * reference to control constraint.
     * @param motIdx Number motor
     * @param reference reference of velocity
     * @return Time to compute this function
     */
    int set_motor_reference(short motIdx, motor_state_t state, motor_control_t reference);

    /**
     * Return state of motor
     * @param motIdx number of motor selected
     * @return state of motor
     */
    inline motor_state_t get_motor_state(short motIdx);
    /**
     * Set state controller for all motors, if DISABLE, set enable motor to zero
     * @param motIdx number motor to update state if -1 set all motor to state
     * @param motor state command
     */
    void set_motor_state(short motIdx, motor_state_t motor);

    /**
     * Convert and check reference for type of law control selected. We have
     * four principal type of control motor:
     *  - Direct control (write direct PWM)
     *  - Position control (move to desired angle)
     *  - Velocity control (move to desired angular velocity)
     *  - Torque control (move to desired torque)
     */
    void MotorTaskController(int argc, int *argv);

    /**
     * Measure velocity from Input Capture and QEI
     * @param motIdx Number motor
     */
    void measureVelocity(short motIdx);
    
    inline void Motor_PWM(short motIdx, int pwm_control);
    
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
     * @param motIdx Number motor
     * @return control evaluation
     */
    inline fractional MotorPID(short motIdx, tPID *pid);
    
    /**
     * If not receive anything velocity messages. Start controlled stop motors
     * @param motIdx Number motor
     * @return start emergency mode or not.
     */
    void Emergency(int argc, int *argv);

#ifdef	__cplusplus
}
#endif

#endif	/* MOTOR_CONTROL_H */