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

#ifndef PARSING_PACKET_H
#define	PARSING_PACKET_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "packet/packet.h"

    void init_hashmap();
    int parse_packet();
    void saveData(information_packet_t* list_send, size_t len, information_packet_t info);
    void sendData(information_packet_t* list_send, size_t len, information_packet_t info);
    packet_t encoder(information_packet_t *list_send, size_t len);
    packet_t encoderSingle(information_packet_t list_send);

    information_packet_t createPacket(unsigned char command, unsigned char option, unsigned char type, abstract_packet_t * packet);
    information_packet_t createDataPacket(unsigned char command, unsigned char type, abstract_packet_t * packet);

#ifdef	__cplusplus
}
#endif

#endif	/* PARSING_PACKET_H */

