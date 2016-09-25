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

#ifndef HIGH_COMM_H
#define	HIGH_COMM_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <or_bus/or_message.h>
    
    /**
     * Save for all standard messages the data in tail and save in controller.
     * Others messages, typical for this board are saved with function
     * save_other_data in file parsing_other_messages.h
     * @param list_send a pointer to buffer to save information from board
     * @param len length of list_send list
     * @param info message to parsing
     */
    packet_information_t save_frame_motion(unsigned char option, unsigned char type, unsigned char command, message_abstract_u message);

    /**
     * Send for all standard messages the data. The information are saved
     * in a information_packet_t by functions createPacket and createDataPacket
     * in tail of this file.
     * @param list_send a pointer to buffer to save information from board
     * @param len length of list_send list
     * @param info message to parsing
     */
    packet_information_t send_frame_motion(unsigned char option, unsigned char type, unsigned char command, message_abstract_u message);


#ifdef	__cplusplus
}
#endif

#endif	/* HIGH_COMM_H */

