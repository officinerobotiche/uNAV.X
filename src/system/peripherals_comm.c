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

#include <xc.h>                 /* Device header file */

#include <or_bus/or_frame.h>

#include "system/peripherals.h"
#include "communication/serial.h"

//TEMP
#include <peripherals/gpio.h>

/******************************************************************************/
/* Parsing functions                                                          */
/******************************************************************************/

packet_information_t save_frame_gpio(unsigned char option, unsigned char type, unsigned char command, message_abstract_u message) {
    int port_name;
    switch (command) {
        case PERIPHERALS_GPIO_SET:
            port_name = (message.gpio.set.name == 'A') ? 0 : 1;
            gpio_setup(port_name, message.gpio.set.number, message.gpio.set.type);
            break;
        case PERIPHERALS_GPIO:
            port_name = (message.gpio.set.name == 'A') ? 0 : 1;
            gpio_set(port_name, BIT_MASK(message.gpio.port.port));
            break;
        case PERIPHERALS_GPIO_ALL:
            port_name = (message.gpio.port.name == 'A') ? 0 : 1;
            gpio_set(port_name, message.gpio.port.port);
            break;
        case PERIPHERALS_SERIAL:
            return CREATE_PACKET_RESPONSE(command, type, Serial_set(message.gpio.serial));
            break;
        default:
            return CREATE_PACKET_NACK(command, type);
            break;
    }
    return CREATE_PACKET_ACK(command, type);
}

packet_information_t send_frame_gpio(unsigned char option, unsigned char type, unsigned char command, message_abstract_u message) {
    int port_name;
    message_abstract_u send;
    switch (command) {
        case PERIPHERALS_GPIO_SET:
            port_name = (message.gpio.set.name == 'A') ? 0 : 1;
            send.gpio.set.type = gpio_config(port_name, message.gpio.set.number);
            break;
        case PERIPHERALS_GPIO:
            port_name = (message.gpio.set.name == 'A') ? 0 : 1;
            if(gpio_config(port_name, message.gpio.set.number) == GPIO_ANALOG) {
                send.gpio.port.port = gpio_get_analog(port_name, message.gpio.port.port);
            } else {
                int data = gpio_get(port_name);
                send.gpio.port.port = REGISTER_MASK_READ(&data, BIT_MASK(message.gpio.port.port));
            }
            break;
        case PERIPHERALS_GPIO_ALL:
            port_name = (message.gpio.port.name == 'A') ? 0 : 1;
            send.gpio.port.port = gpio_get(port_name);
            break;
        case PERIPHERALS_SERIAL:
            send.gpio.serial = Serial_get(0);
            break;
        default:
            return CREATE_PACKET_NACK(command, type);
            break;
    }
    return CREATE_PACKET_DATA(command, type, send);
};
