/* 
 * File:   packet.h
 * Author: raffaello
 *
 * Created on 12 giugno 2013, 15.29
 */

#ifndef PACKET_H
#define	PACKET_H

#ifdef	__cplusplus
extern "C" {
#endif

#define MAX_TX_BUFF 200
#define MAX_RX_BUFF 200

#define REQUEST 'R'
#define CHANGE 'C'
#define ACK 'K'
#define NACK 'N'

    //

#define PID_CONTROL_L 'A'
#define PID_CONTROL_R 'B'
#define MOTOR_L 'a'
#define MOTOR_R 'b'
#define COORDINATE 'C'
#define PARAMETER 'P'
#define CONSTRAINT 'T'
#define TIME_PROCESS 't'
#define PRIORITY_PROCESS 'p'
#define FRQ_PROCESS 'f'
#define VELOCITY 'V'
#define VELOCITY_MIS 'M'
#define ENABLE 'E'
#define ERROR_SERIAL 'e'

#define SERVICES 's'
#define SERVICE_BUFF 10
#define RESET '*'
#define VERSION_CODE 'v'

    typedef struct error_pkg {
        int16_t number;
        int16_t repeat;
    } error_pkg_t;
#define LNG_ERROR_PKG sizeof(error_pkg_t)

    typedef struct services {
        char command;
        unsigned char buffer[SERVICE_BUFF];
    } services_t;
#define LNG_SERVICES sizeof(services_t)

    typedef struct time {
        int16_t idle;
        int16_t pid_l;
        int16_t pid_r;
        int16_t velocity;
        int16_t dead_reckoning;
        int16_t parse_packet;
    } process_t;
#define LNG_PROCESS sizeof(process_t)

    typedef struct constraint {
        float max_left;
        float max_right;
    } constraint_t;
#define LNG_CONSTRAINT sizeof(motor_t)

    typedef struct motor {
        int16_t rifer_vel;
        int16_t control_vel;
        int16_t measure_vel;
        int16_t current;
    } motor_t;
#define LNG_MOTOR sizeof(motor_t)

    typedef struct pid {
        float kp;
        float ki;
        float kd;
    } pid_control_t;
#define LNG_PID_CONTROL sizeof(pid_control_t)

    typedef struct coordinate {
        float x;
        float y;
        float theta;
        float space;
    } coordinate_t;
#define LNG_COORDINATE sizeof(coordinate_t)

    typedef struct parameter {
        float radius_r;
        float radius_l;
        float wheelbase;
        float k_vel_r;
        float k_vel_l;
        float k_ang_r;
        float k_ang_l;
        float sp_min;
        int16_t step_timer;
        int16_t int_tm_mill;
    } parameter_t;
#define LNG_PARAMETER sizeof(parameter_t)

    typedef struct velocity {
        float v;
        float w;
    } velocity_t;
#define LNG_VELOCITY sizeof(velocity_t)

    typedef uint8_t enable_motor_t;
#define LNG_ENABLE sizeof(enable_motor_t) + 1

    typedef struct packet_data {
        unsigned int length;
        unsigned char buffer[MAX_RX_BUFF];
        unsigned int time;
    } packet_t;

    typedef union abstract_packet {
        unsigned char buffer[MAX_RX_BUFF];
        pid_control_t pid;
        coordinate_t coordinate;
        parameter_t parameter;
        velocity_t velocity;
        enable_motor_t enable;
        motor_t motor;
        constraint_t constraint;
        process_t process;
        services_t services;
        error_pkg_t error_pkg;
    } abstract_packet_t;
    typedef abstract_packet_t* Ptr_abstract_packet;

    typedef struct information_packet {
        unsigned char command;
        unsigned char option;
        abstract_packet_t packet;
    } information_packet_t;

    typedef packet_t* Ptr_packet;


#ifdef	__cplusplus
}
#endif

#endif	/* PACKET_H */

