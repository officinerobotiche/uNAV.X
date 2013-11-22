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


#include <stdint.h>        /* Includes uint16_t definition                    */
#include <stdbool.h>       /* Includes true/false definition                  */

#include "system.h"        /* System funct/params, like osc/peripheral config */
#include "user.h"          /* User funct/params, such as InitApp              */
#include "serial.h"
#include "motors_PID.h"
#include "high_level_control.h"
#include "parsing_packet.h"

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

/* i.e. uint16_t <variable_name>; */

/******************************************************************************/
/* Main Program                                                               */

/******************************************************************************/

int16_t main(void) {

    /* Configure the oscillator for the device */
    ConfigureOscillator();

    /* Initialize hashmap packet */
    init_hashmap();
    /* Initialize variables for robots */
    init_buff_serial_error();
    init_parameter();
    init_process();
    init_pid_control();

    /* Initialize pid controllers */
    update_pid_l();
    update_pid_r();

    /* Initialize dead reckoning */
    init_coordinate();

    /* Initialize IO ports and peripherals */
    InitApp();

    while (1) {

    }
}
