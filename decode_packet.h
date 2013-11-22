/* 
 * File:   decode_packet.h
 * Author: raffaello
 *
 * Created on 22 novembre 2013, 15.41
 */

#ifndef DECODE_PACKET_H
#define	DECODE_PACKET_H

#ifdef	__cplusplus
extern "C" {
#endif

    void saveOtherData(information_packet_t* list_send, size_t len, information_packet_t info);
    void sendOtherData(information_packet_t* list_send, size_t len, information_packet_t info);


#ifdef	__cplusplus
}
#endif

#endif	/* DECODE_PACKET_H */

