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

#ifndef SERIAL_H
#define	SERIAL_H

#ifdef	__cplusplus
extern "C" {
#endif

    #include <serial/or_message.h>
    #include <serial/or_frame.h>
    
    //Dimension of list messages to decode in a packet
    #define BUFFER_LIST_PARSING 10

    /**
     * Send serial message to uart
     * @param header
     * @param packet
     */
    void serial_send(char header, packet_t packet);
    
    /**
     * In a packet we have more messages. A typical data packet
     * have this structure:
     * -------------------------- ---------------------------- -----------------------
     * | Length | CMD | DATA ... | Length | CMD | INFORMATION |Length | CMD | ... ... |
     * -------------------------- ---------------------------- -----------------------
     *    1        2 -> length    length+1 length+2 length+3   ...
     * It is possible to have different type of messages:
     * * Message with data (D)
     * * Message with state information:
     *      * (R) request data
     *      * (A) ack
     *      * (N) nack
     * We have three parts to elaborate and send a new packet (if required)
     * 1. [SAVING] The first part of this function split packets in a
     * list of messages to compute.
     * 2. [COMPUTE] If message have a data in tail, start compute and return a
     * new ACK or NACK message to append in a new packet. If is a request
     * message (R), the new message have in tail the data required.
     * 3. [SEND] Encoding de messages and transform in a packet to send.
     * *This function is a long function*
     * @return time to compute parsing packet
     */
    int parse_packet();
    
    /**
     * Save for all standard messages the data in tail and save in controller.
     * Others messages, typical for this board are saved with function
     * save_other_data in file parsing_other_messages.h
     * @param list_send a pointer to buffer to save information from board
     * @param len length of list_send list
     * @param info message to parsing
     */
    void save_frame_system(packet_information_t* list_send, size_t* len, packet_information_t* info);

    /**
     * Send for all standard messages the data. The information are saved
     * in a information_packet_t by functions createPacket and createDataPacket
     * in tail of this file.
     * @param list_send a pointer to buffer to save information from board
     * @param len length of list_send list
     * @param info message to parsing
     */
    void send_frame_system(packet_information_t* list_send, size_t* len, packet_information_t* info);

#ifdef	__cplusplus
}
#endif

#endif	/* SERIAL_H */

