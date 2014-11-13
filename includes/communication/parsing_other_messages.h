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

#ifndef PARSING_OTHER_MESSAGES_H
#define	PARSING_OTHER_MESSAGES_H

#ifdef	__cplusplus
extern "C" {
#endif

    /**
     * Similar to saveData (in parsing_mesages.h) this function save messages
     * from packet recived.
     * @param list_send a pointer to buffer to save information from board
     * @param len length of list_send list
     * @param info message to parsing
     */
    void saveOtherData(information_packet_t* list_send, size_t len, information_packet_t* info);

    /**
     * Similar to sendData (in parsing_messages.h) this function send messages
     * about unav board.
     * @param list_send a pointer to buffer to save information from board
     * @param len length of list_send list
     * @param info message to parsing
     */
    void sendOtherData(information_packet_t* list_send, size_t len, information_packet_t* info);


#ifdef	__cplusplus
}
#endif

#endif	/* PARSING_OTHER_MESSAGES_H */

