/* 
 * File:   parsing_packet.h
 * Author: raffaello
 *
 * Created on 12 giugno 2013, 16.24
 */

#ifndef PARSING_PACKET_H
#define	PARSING_PACKET_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "packet.h"

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

