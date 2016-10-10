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

//Split motor and command
peripheral_gpio_map_t peripheral;

/******************************************************************************/
/* Parsing functions                                                          */
/******************************************************************************/

packet_information_t save_frame_gpio(unsigned char option, unsigned char type, unsigned char command, message_abstract_u message) {
    peripheral.message = command;
    gpio_port_t port;
    switch (peripheral.bitset.command) {
        case PERIPHERALS_GPIO_SET:
            gpio_setup(peripheral.bitset.port, message.gpio.set.port.port, message.gpio.set.type);
            break;
        case PERIPHERALS_GPIO_DIGITAL:
            port.len = message.gpio.port.len;
            port.port = message.gpio.port.port;
            gpio_set(peripheral.bitset.port, port);
            break;
//        case PERIPHERALS_GPIO:
//            port_name = (message.gpio.set.name == 'A') ? 0 : 1;
//            gpio_set(port_name, BIT_MASK(message.gpio.port.port));
//            break;
//        case PERIPHERALS_SERIAL:
//            return CREATE_PACKET_RESPONSE(command, type, Serial_set(message.gpio.serial));
//            break;
        default:
            return CREATE_PACKET_NACK(command, type);
            break;
    }
    return CREATE_PACKET_ACK(command, type);
}

packet_information_t send_frame_gpio(unsigned char option, unsigned char type, unsigned char command, message_abstract_u message) {
    message_abstract_u send;
    peripheral.message = command;
    gpio_port_t port;
//    peripherals_gpio_set_t setup;
    switch (peripheral.bitset.command) {
//        case PERIPHERALS_GPIO_SET:
//            setup.type = gpio_config(peripheral.bitset.port, message.gpio.number);
//            setup.port.port = message.gpio.number;
//            send.gpio.set = setup;
//            break;
        case PERIPHERALS_GPIO:
            if(gpio_config(peripheral.bitset.port, message.gpio.number) == GPIO_ANALOG) {
                send.gpio.pin = gpio_get_analog(peripheral.bitset.port, message.gpio.port.port);
            } else {
                port = gpio_get(peripheral.bitset.port);
                send.gpio.pin = REGISTER_MASK_READ(&port.port, BIT_MASK(message.gpio.port.port));
            }
            break;
        case PERIPHERALS_GPIO_DIGITAL:
            port = gpio_get(peripheral.bitset.port);
            send.gpio.port.port = port.port;
            send.gpio.port.len = port.len;
            break;
        case PERIPHERALS_SERIAL:
            send.gpio.serial = Serial_get(peripheral.bitset.port);
            break;
        default:
            return CREATE_PACKET_NACK(command, type);
            break;
    }
    return CREATE_PACKET_DATA(command, type, send);
};
