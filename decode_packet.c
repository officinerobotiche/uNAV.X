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

#include "high_level_control.h"
#include "motors_PID.h"
#include "user.h"
#include "system.h"

// From motors PID
extern parameter_motors_t parameter_motors;
extern constraint_t constraint;
extern velocity_t vel_rif, vel_mis;
extern pid_control_t pid_left, pid_right;
extern enable_motor_t enable_motors;
extern motor_t motor_left, motor_right;

// From high level control
extern coordinate_t coordinate;
extern bool coord_busy;

/******************************************************************************/
/* Computation functions                                                      */
/******************************************************************************/

void saveOtherData(information_packet_t* list_send, size_t len, information_packet_t info) {
    if (info.type == HASHMAP_MOTION)
        switch (info.command) {
            case PID_CONTROL_L:
                pid_left = info.packet.pid;
                update_pid_l();
                list_send[len] = createPacket(info.command, ACK, info.type, NULL);
                break;
            case PID_CONTROL_R:
                pid_right = info.packet.pid;
                update_pid_r();
                list_send[len] = createPacket(info.command, ACK, info.type, NULL);
                break;
            case COORDINATE:
                coordinate = info.packet.coordinate;
                update_coord();
                list_send[len] = createPacket(info.command, ACK, info.type, NULL);
                break;
            case PARAMETER_MOTORS:
                parameter_motors = info.packet.parameter_motors;
                update_parameter();
                list_send[len] = createPacket(info.command, ACK, info.type, NULL);
                break;
            case CONSTRAINT:
                constraint = info.packet.constraint;
                list_send[len] = createPacket(info.command, ACK, info.type, NULL);
                break;
            case VELOCITY:
                vel_rif = info.packet.velocity;
                list_send[len] = createPacket(info.command, ACK, info.type, NULL);
                break;
            case ENABLE:
                enable_motors = info.packet.enable;
                list_send[len] = createPacket(info.command, ACK, info.type, NULL);
                break;
            case MOTOR_L:
            case MOTOR_R:
            case VELOCITY_MIS:
                list_send[len] = createPacket(info.command, NACK, info.type, NULL);
                break;
            default:
                list_send[len] = createPacket(info.command, NACK, info.type, NULL);
                break;
        }
}

void sendOtherData(information_packet_t* list_send, size_t len, information_packet_t info) {
    abstract_packet_t send;
    if (info.type == HASHMAP_MOTION)
        switch (info.command) {
            case PID_CONTROL_L:
                send.pid = pid_left;
                list_send[len] = createDataPacket(info.command, info.type, &send);
                break;
            case PID_CONTROL_R:
                send.pid = pid_right;
                list_send[len] = createDataPacket(info.command, info.type, &send);
                break;
            case COORDINATE:
                send.coordinate = coordinate;
                list_send[len] = createDataPacket(info.command, info.type, &send);
                break;
            case PARAMETER_MOTORS:
                send.parameter_motors = parameter_motors;
                list_send[len] = createDataPacket(info.command, info.type, &send);
                break;
            case CONSTRAINT:
                send.constraint = constraint;
                list_send[len] = createDataPacket(info.command, info.type, &send);
                break;
            case VELOCITY:
                send.velocity = vel_rif;
                list_send[len] = createDataPacket(info.command, info.type, &send);
                break;
            case ENABLE:
                send.enable = MOTOR_ENABLE1 && MOTOR_ENABLE2;
                list_send[len] = createDataPacket(info.command, info.type, &send);
                break;
            case MOTOR_L:
                send.motor = motor_left;
                list_send[len] = createDataPacket(info.command, info.type, &send);
                break;
            case MOTOR_R:
                send.motor = motor_right;
                list_send[len] = createDataPacket(info.command, info.type, &send);
                break;
            case VELOCITY_MIS:
                send.velocity = vel_mis;
                list_send[len] = createDataPacket(info.command, info.type, &send);
                break;
            default:
                list_send[len] = createPacket(info.command, NACK, info.type, NULL);
                break;
        }
}
