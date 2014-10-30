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

#ifndef PARSING_MESSAGES_H
#define	PARSING_MESSAGES_H

#ifdef	__cplusplus
extern "C" {
#endif

    #include "packet/packet.h"

    //Dimension of list messages to decode in a packet
    #define BUFFER_LIST_PARSING 10

    /**
     * Init hashmap for decode messages
     * Load all hashmaps from packet/packet.h and packet/unav.h
     */
    void init_hashmap();
    
    /**
     * Save for all standard messages the data in tail and save in controller.
     * Others messages, tipical for this board are saved with function
     * save_other_data in file parsing_other_messages.h
     * @param list_send a pointer to buffer to save information from board
     * @param len length of list_send list
     * @param info message to parsing
     */
    void saveData(information_packet_t* list_send, size_t len, information_packet_t info);

    /**
     * Send for all standard messages the data. The information are saved
     * in a information_packet_t by functions createPacket and createDataPacket
     * in tail of this file.
     * @param list_send a pointer to buffer to save information from board
     * @param len length of list_send list
     * @param info message to parsing
     */
    void sendData(information_packet_t* list_send, size_t len, information_packet_t info);

    /**
     * In a packet we have more messages. A typical data packet
     * have this struct:
     * -------------------------- ---------------------------- -----------------------
     * | Length | CMD | DATA ... | Length | CMD | INFORMATION |Length | CMD | ... ... |
     * -------------------------- ---------------------------- -----------------------
     *    1        2 -> length    length+1 length+2 length+3   ...
     * It is possibile to have different type of messages:
     * * Message with data (D)
     * * Message witn state information:
     *      * (R) request data
     *      * (A) ack
     *      * (N) nack
     * We have tre parts to elaborate and send a new packet (if required)
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
     * Get a list of messages to transform in a packet for serial communication.
     * This function create a new packet and copy with UNION buffer_packet_u and
     * finally put chars convertion in to buffer.
     * @param list_send pointer of list with messages to send
     * @param len length of list_send list
     * @return a packet_t with all data to send
     */
    packet_t encoder(information_packet_t *list_send, size_t len);

    /**
     * Get an information_packet to convert in a buffer of char to put
     * in a packet_t data.
     * @param list_send information_packet_t to send
     * @return a packet_t with data to send
     */
    packet_t encoderSingle(information_packet_t list_send);

    /**
     * Create a information_packet_t associated a message listed in
     * abstract_message_t (see packet/packet.h for all listed messages).
     * Finally add information about message.
     * @param command type of message to send
     * @param option information about this message
     * @param type type of message
     * @param packet abstract_message to convert in a information_packet
     * @return information_packet ready to send
     */
    information_packet_t createPacket(unsigned char command, unsigned char option, unsigned char type, abstract_message_u * packet);
    
    /**
     * Create an information packet for a message with data (D).
     * This function use createPacket for create information_packet
     * @param command information about this message
     * @param type type of command to send
     * @param packet abstract_message to convert in a information_packet
     * @return information_packet ready to send
     */
    information_packet_t createDataPacket(unsigned char command, unsigned char type, abstract_message_u * packet);

#ifdef	__cplusplus
}
#endif

#endif	/* PARSING_MESSAGES_H */

