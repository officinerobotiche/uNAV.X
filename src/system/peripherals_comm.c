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

/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

/* Device header file */
#if defined(__XC16__)
#include <xc.h>
#elif defined(__C30__)
#if defined(__dsPIC33E__)
#include <p33Exxxx.h>
#elif defined(__dsPIC33F__)
#include <p33Fxxxx.h>
#endif
#endif

#include <serial/or_frame.h>

#include "system/peripherals.h"

//TEMP
#include <peripherals/gpio.h>

/******************************************************************************/
/* Parsing functions                                                          */
/******************************************************************************/

void save_frame_gpio(packet_information_t* list_send, size_t* len, packet_information_t* info) {
    switch (info->command) {
        case PERIPHERALS_GPIO_SET:
            
            break;
        case PERIPHERALS_GPIO_ALL:
            gpio_set(1, info->message.gpio.port);
            list_send[(*len)++] = CREATE_PACKET_ACK(info->command, info->type);
            break;
        default:
            list_send[(*len)++] = CREATE_PACKET_NACK(info->command, info->type);
            break;
    }
}

void send_frame_gpio(packet_information_t* list_send, size_t* len, packet_information_t* info) {
    
};
