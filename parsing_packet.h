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

    int parse_packet(void);
    information_packet_t decode_single_pkg(Ptr_packet send, char command, unsigned char* Buffer, unsigned int position);
    information_packet_t addChangePacket(Ptr_packet send, char command, unsigned char* Buffer, unsigned int position, unsigned int length, Ptr_abstract_packet packet);
    information_packet_t addPacket(Ptr_packet send, unsigned char command, unsigned char option, Ptr_abstract_packet packet);
    information_packet_t addRequestPacket(Ptr_packet send, unsigned char command, Ptr_abstract_packet pkg);
    information_packet_t addInformationPacket(Ptr_packet send, unsigned char command, unsigned char option);
    information_packet_t buildRequestPacket(Ptr_packet send, unsigned char command, const unsigned int length, Ptr_abstract_packet packet);




#ifdef	__cplusplus
}
#endif

#endif	/* PARSING_PACKET_H */

