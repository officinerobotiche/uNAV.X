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

    /**
     * Init hashmap for decode packets
     */
    void init_hashmap();
    
    /**
     * on packet:
     * -------------------------- ---------------------------- -----------------------
     * | Length | CMD | DATA ... | Length | CMD | INFORMATION |Length | CMD | ... ... |
     * -------------------------- ---------------------------- -----------------------
     *    1        2 -> length    length+1 length+2 length+3   ...
     *
     * @return
     */
    int parse_packet();
    
    /**
     * 
     * @param list_send
     * @param len
     * @param info
     */
    void saveData(information_packet_t* list_send, size_t len, information_packet_t info);
    
    /**
     * 
     * @param list_send
     * @param len
     * @param info
     */
    void sendData(information_packet_t* list_send, size_t len, information_packet_t info);
    
    /**
     * 
     * @param list_send
     * @param len
     * @return 
     */
    packet_t encoder(information_packet_t *list_send, size_t len);

    /**
     *
     * @param list_send
     * @return
     */
    packet_t encoderSingle(information_packet_t list_send);

    /**
     *
     * @param command
     * @param option
     * @param type
     * @param packet
     * @return
     */
    information_packet_t createPacket(unsigned char command, unsigned char option, unsigned char type, abstract_packet_t * packet);
    
    /**
     * 
     * @param command
     * @param type
     * @param packet
     * @return 
     */
    information_packet_t createDataPacket(unsigned char command, unsigned char type, abstract_packet_t * packet);

#ifdef	__cplusplus
}
#endif

#endif	/* PARSING_PACKET_H */

