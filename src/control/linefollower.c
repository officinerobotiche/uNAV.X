/* 
 * File:   linefollower.c
 * Author: Marco
 *
 * Created on 25 aprile 2015, 10.36
 */

#include <stdint.h>        /* Includes uint16_t definition                    */
#include <stdbool.h>       /* Includes true/false definition                  */

#include "control/high_level_control.h"
#include "control/motors_PID.h"

extern velocity_t vel_rif, vel_mis;
extern unsigned int counter_alive[NUM_MOTORS];

void linefollowing()
{
    //Reset time emergency
    counter_alive[0] = 0;
    counter_alive[1] = 0;
               
    // Our Hello World!!!! :)
 
    vel_rif.v = 0.2;
    
}
