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
#include "communication/serial.h"

//TEMP
#include <peripherals/gpio.h>

/******************************************************************************/
/* Parsing functions                                                          */
/******************************************************************************/

void save_frame_gpio(packet_information_t* list_send, size_t* len, packet_information_t* info) {
    int port_name;
    switch (info->command) {
        case PERIPHERALS_GPIO_SET:
            port_name = (info->message.gpio.set.name == 'A') ? 0 : 1;
            gpio_setup(port_name, info->message.gpio.set.number, info->message.gpio.set.type);
            list_send[(*len)++] = CREATE_PACKET_ACK(info->command, info->type);
            break;
        case PERIPHERALS_GPIO:
            port_name = (info->message.gpio.set.name == 'A') ? 0 : 1;
            gpio_set(port_name, BIT_MASK(info->message.gpio.port.port));
            list_send[(*len)++] = CREATE_PACKET_ACK(info->command, info->type);
            break;
        case PERIPHERALS_GPIO_ALL:
            port_name = (info->message.gpio.port.name == 'A') ? 0 : 1;
            gpio_set(port_name, info->message.gpio.port.port);
            list_send[(*len)++] = CREATE_PACKET_ACK(info->command, info->type);
            break;
        case PERIPHERALS_SERIAL:
            list_send[(*len)++] = CREATE_PACKET_RESPONSE(info->command, info->type, Serial_set(info->message.gpio.serial));
            break;
        default:
            list_send[(*len)++] = CREATE_PACKET_NACK(info->command, info->type);
            break;
    }
}

void send_frame_gpio(packet_information_t* list_send, size_t* len, packet_information_t* info) {
    int port_name;
    message_abstract_u send;
    switch (info->command) {
        case PERIPHERALS_GPIO_SET:
            port_name = (info->message.gpio.set.name == 'A') ? 0 : 1;
            send.gpio.set.type = gpio_config(port_name, info->message.gpio.set.number);
            list_send[(*len)++] = CREATE_PACKET_DATA(info->command, info->type, send);
            break;
        case PERIPHERALS_GPIO:
            port_name = (info->message.gpio.set.name == 'A') ? 0 : 1;
            if(gpio_config(port_name, info->message.gpio.set.number) == GPIO_ANALOG) {
                send.gpio.port.port = gpio_get_analog(port_name, info->message.gpio.port.port);
            } else {
                int data = gpio_get(port_name);
                send.gpio.port.port = REGISTER_MASK_READ(&data, BIT_MASK(info->message.gpio.port.port));
            }
            list_send[(*len)++] = CREATE_PACKET_DATA(info->command, info->type, send);
            break;
        case PERIPHERALS_GPIO_ALL:
            port_name = (info->message.gpio.port.name == 'A') ? 0 : 1;
            send.gpio.port.port = gpio_get(port_name);
            list_send[(*len)++] = CREATE_PACKET_DATA(info->command, info->type, send);
            break;
        case PERIPHERALS_SERIAL:
            send.gpio.serial = Serial_get(0);
            list_send[(*len)++] = CREATE_PACKET_DATA(info->command, info->type, send);
            break;
        default:
            list_send[(*len)++] = CREATE_PACKET_NACK(info->command, info->type);
            break;
    }
};
