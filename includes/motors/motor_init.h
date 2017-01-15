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
     * Initialization motor
     */
    void Motor_Init(LED_controller_t* led_controller);
    /**
     * 
     * @param obj
     * @param type
     * @param command
     * @param packet
     */
    void OR_BUS_FRAME_decoder_motor(void* obj, OR_BUS_FRAME_type_t type, 
        OR_BUS_FRAME_command_t command, OR_BUS_FRAME_packet_t *packet);

#ifdef	__cplusplus
}
#endif

#endif	/* MOTOR_INIT_H */

