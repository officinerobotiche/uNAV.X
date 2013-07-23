/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

/* Device header file */
#if defined(__XC16__)
#include <xc.h>
#elif defined(__C30__)
#if defined(__dsPIC33E__)
#include <p33Exxxx.h>
#elif defined(__dsPIC33F__)
#include <p33Fxxxx.h>
#endif
#endif

#include <stdint.h>        /* Includes uint16_t definition   */
#include <stdbool.h>       /* Includes true/false definition */
#include <string.h>

#include "parsing_packet.h"

#include "serial.h"

#include "motors_PID.h"
#include "user.h"
#include "system.h"


// From Interrupt
extern volatile process_t time, priority, frequency;

// From serial
extern packet_t receive_pkg;

// From motors PID
extern parameter_t parameter;
extern constraint_t constraint;
extern velocity_t vel_rif, vel_mis;
extern pid_control_t pid_left, pid_right;
extern enable_motor_t enable_motors;
extern motor_t motor_left, motor_right;

// From high level control
extern coordinate_t coordinate;


/******************************************************************************/
/* Parsing functions                                                          */

/******************************************************************************/

int parse_packet(void) {
    int i;
    unsigned int t = TMR1; // Timing function
    packet_t send, packet;
    send.length = 0;
    memcpy(&packet, &receive_pkg, sizeof (packet));
    for (i = 0; i < packet.length; i++) {
        unsigned char option;
        if (packet.buffer[i] > 1)
            option = CHANGE;
        else
            option = packet.buffer[i + 2];
        switch (option) {
            case CHANGE:
                decode_single_pkg(&send, packet.buffer[i + 1], packet.buffer, i + 2);
                break;
            case REQUEST:
            case ACK:
            case NACK:
                addPacket(&send, packet.buffer[i + 1], packet.buffer[i + 2], NULL);
                break;
            default:
                break;
        }
        i += packet.buffer[i] + 1;
    }
    pkg_send(send);
    return TMR1 - t; // Time of esecution
}

information_packet_t decode_single_pkg(Ptr_packet send, char command, unsigned char* Buffer, unsigned int position) {
    information_packet_t packet;
    Ptr_abstract_packet ptr_packet_send;
    abstract_packet_t packet_send;
    services_t service;
    unsigned char process_return;
    switch (command) {
        case MOTOR_L:
            // Impossible to change measure motor left
            return addPacket(send, command, NACK, NULL);
            break;
        case MOTOR_R:
            // Impossible to change measure motor left
            return addPacket(send, command, NACK, NULL);
            break;
        case PID_CONTROL_L:
            ptr_packet_send = (Ptr_abstract_packet) & pid_left;
            packet = addChangePacket(send, command, Buffer, position, LNG_PID_CONTROL, ptr_packet_send);
            update_pid_l();
            return packet;
            break;
        case PID_CONTROL_R:
            ptr_packet_send = (Ptr_abstract_packet) & pid_right;
            packet = addChangePacket(send, command, Buffer, position, LNG_PID_CONTROL, ptr_packet_send);
            update_pid_r();
            return packet;
            break;
        case COORDINATE:
            ptr_packet_send = (Ptr_abstract_packet) & coordinate;
            return addChangePacket(send, command, Buffer, position, LNG_COORDINATE, ptr_packet_send);
            break;
        case CONSTRAINT:
            ptr_packet_send = (Ptr_abstract_packet) & constraint;
            return addChangePacket(send, command, Buffer, position, LNG_CONSTRAINT, ptr_packet_send);
            break;
        case PARAMETER:
            ptr_packet_send = (Ptr_abstract_packet) & parameter;
            packet = addChangePacket(send, command, Buffer, position, LNG_PARAMETER, ptr_packet_send);
            update_parameter();
            return packet;
            break;
        case VELOCITY:
            ptr_packet_send = (Ptr_abstract_packet) & vel_rif;
            return addChangePacket(send, command, Buffer, position, LNG_VELOCITY, ptr_packet_send);
            break;
        case VELOCITY_MIS:
            // Impossible to change velocity measure
            return addPacket(send, command, NACK, NULL);
            break;
        case ENABLE:
            ptr_packet_send = (Ptr_abstract_packet) & enable_motors;
            return addChangePacket(send, command, Buffer, position, LNG_ENABLE, ptr_packet_send);
            break;
        case TIME_PROCESS:
            return addPacket(send, command, NACK, NULL);
            break;
        case PRIORITY_PROCESS:
            process_return = update_priority();
            return addPacket(send, command, process_return, NULL);
            break;
        case FRQ_PROCESS:
            process_return = update_frequency();
            return addPacket(send, command, process_return, NULL);
            break;
        case SERVICES:
            //Save packet
            ptr_packet_send = (Ptr_abstract_packet) & service;
            addChangePacket(send, command, Buffer, position, LNG_SERVICES, ptr_packet_send);
            //Esecution service
            packet_send.services = services(service);
            //Send return service packet
            return buildRequestPacket(send, command, LNG_SERVICES, &packet_send);
            break;
        default:
            pkg_error(ERROR_PKG);
            return addPacket(send, command, NACK, NULL);
            break;
    }
}

information_packet_t addChangePacket(Ptr_packet send, char command, unsigned char* Buffer, unsigned int position, unsigned int length, Ptr_abstract_packet packet) {
    if (packet != NULL) {
        memcpy(packet->buffer, Buffer + (position * sizeof (unsigned char)), length);
        return addPacket(send, command, ACK, NULL);
    } else return addPacket(send, command, NACK, NULL);
}

information_packet_t addPacket(Ptr_packet send, unsigned char command, unsigned char option, Ptr_abstract_packet packet) {
    switch (option) {
        case REQUEST:
        case CHANGE:
            return addRequestPacket(send, command, packet);
            break;
        case ACK:
        case NACK:
            return addInformationPacket(send, command, option);
            break;
        default:
            pkg_error(ERROR_OPTION);
            return addPacket(send, command, NACK, NULL);
            break;
    }
}

information_packet_t addRequestPacket(Ptr_packet send, unsigned char command, Ptr_abstract_packet pkg) {
    abstract_packet_t packet;
    switch (command) {
        case MOTOR_L:
            packet.motor = motor_left;
            return buildRequestPacket(send, command, LNG_MOTOR, &packet);
            break;
        case MOTOR_R:
            packet.motor = motor_right;
            return buildRequestPacket(send, command, LNG_MOTOR, &packet);
            break;
        case PID_CONTROL_L:
            packet.pid = pid_left;
            return buildRequestPacket(send, command, LNG_PID_CONTROL, &packet);
            break;
        case PID_CONTROL_R:
            packet.pid = pid_right;
            return buildRequestPacket(send, command, LNG_PID_CONTROL, &packet);
            break;
        case COORDINATE:
            packet.coordinate = coordinate;
            //            protectedMemcpy(DEAD_RECK_ENABLE, &packet.coordinate, &coordinate, sizeof (coordinate));
            return buildRequestPacket(send, command, LNG_COORDINATE, &packet);
            break;
        case PARAMETER:
            packet.parameter = parameter;
            return buildRequestPacket(send, command, LNG_PARAMETER, &packet);
            break;
        case CONSTRAINT:
            packet.constraint = constraint;
            return buildRequestPacket(send, command, LNG_CONSTRAINT, &packet);
            break;
        case VELOCITY:
            packet.velocity = vel_rif;
            return buildRequestPacket(send, command, LNG_VELOCITY, &packet);
            break;
        case VELOCITY_MIS:
            //            memcpy(&packet.velocity, &vel_mis, sizeof (velocity_t));
            packet.velocity = vel_mis;
            return buildRequestPacket(send, command, LNG_VELOCITY, &packet);
            break;
        case ENABLE:
            packet.enable = MOTOR_ENABLE1 && MOTOR_ENABLE2;
            return buildRequestPacket(send, command, LNG_ENABLE, &packet);
            break;
        case TIME_PROCESS:
            packet.process = time;
            return buildRequestPacket(send, command, LNG_PROCESS, &packet);
            break;
        case PRIORITY_PROCESS:
            packet.process = priority;
            return buildRequestPacket(send, command, LNG_PROCESS, &packet);
            break;
        case FRQ_PROCESS:
            packet.process = frequency;
            return buildRequestPacket(send, command, LNG_PROCESS, &packet);
            break;
        case SERVICES:
            //TODO
            return addPacket(send, command, NACK, NULL);
            break;
        default:
            pkg_error(ERROR_CREATE_PKG);
            return addPacket(send, command, NACK, NULL);
            break;
    }
}

information_packet_t addInformationPacket(Ptr_packet send, unsigned char command, unsigned char option) {
    send->buffer[send->length] = 1;
    send->buffer[send->length + 1] = command;
    send->buffer[send->length + 2] = option;
    send->length += 3;
    information_packet_t option_pkg;
    option_pkg.command = command;
    option_pkg.option = option;
    return option_pkg;
}

information_packet_t buildRequestPacket(Ptr_packet send, unsigned char command, const unsigned int length, Ptr_abstract_packet packet) {
    if (packet != NULL) {
        information_packet_t request_packet;
        send->buffer[send->length] = length;
        send->buffer[send->length + 1] = command;
        memcpy(send->buffer + (send->length + 2) * sizeof (unsigned char), packet->buffer, length);
        send->length += length + 2;
        request_packet.command = command;
        request_packet.option = REQUEST;
        memcpy(&request_packet.packet.buffer, packet->buffer, length);
        return request_packet;
    } else {
        return addInformationPacket(send, command, REQUEST);
    }
}