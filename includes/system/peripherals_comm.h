/* 
 * File:   peripherals_comm.h
 * Author: Raffaello
 *
 * Created on July 31, 2015, 2:58 PM
 */

#ifndef PERIPHERALS_COMM_H
#define	PERIPHERALS_COMM_H

#ifdef	__cplusplus
extern "C" {
#endif

    #include <or_bus/or_frame.h>
    
void save_frame_gpio(packet_information_t* list_send, size_t* len, packet_information_t* info);

void send_frame_gpio(packet_information_t* list_send, size_t* len, packet_information_t* info);


#ifdef	__cplusplus
}
#endif

#endif	/* PERIPHERALS_COMM_H */

