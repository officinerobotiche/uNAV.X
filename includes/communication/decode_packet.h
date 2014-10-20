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

#ifndef DECODE_PACKET_H
#define	DECODE_PACKET_H

#ifdef	__cplusplus
extern "C" {
#endif

    /**
     *
     * @param list_send
     * @param len
     * @param info
     */
    void saveOtherData(information_packet_t* list_send, size_t len, information_packet_t info);

    /**
     * 
     * @param list_send
     * @param len
     * @param info
     */
    void sendOtherData(information_packet_t* list_send, size_t len, information_packet_t info);


#ifdef	__cplusplus
}
#endif

#endif	/* DECODE_PACKET_H */

