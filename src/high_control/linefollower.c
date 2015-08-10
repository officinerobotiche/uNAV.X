/*
 * Copyright (C) 2014 Officine Robotiche
 * Authors: Marco Fabbri, Mauro Soligo, Raffaello Bonghi
 * email:  {marco.fabbri, mauro.soligo, raffaello.bonghi}@officinerobotiche.it
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

/*****************************************************************************/
/* Files to Include                                                          */
/*****************************************************************************/

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

#include "high_control/manager.h"

#include "communication/I2c.h"
#include <peripherals/i2c/i2c.h>

#include "high_control/linefollower.h"

#define INPUT       1
#define OUTPUT      0

#define PORT_IO0    _TRISA9
#define PORT_IO1    _TRISC0
#define PORT_IO2    _TRISC1
#define PORT_IO3    _TRISC2
#define PORT_IO4    _TRISC3
#define PORT_IO5    _TRISA4
#define PORT_IO6    _TRISB4

#define PIN_IO0     PORTAbits.RA9
#define PIN_IO1     PORTCbits.RC0       
#define PIN_IO2     PORTCbits.RC1
#define PIN_IO3     PORTCbits.RC2
#define PIN_IO4     PORTCbits.RC3
#define PIN_IO5     PORTAbits.RA4
#define PIN_IO6     PORTBbits.RB4

/*****************************************************************************/
/* Global Variable Declaration                                               */
/*****************************************************************************/

linesensor_t line_sensor;

int test_linefollower;
int discharge_timer;

/*****************************************************************************/
/* User Functions                                                            */
/*****************************************************************************/

void IRsensor_Init(void) {
    
}

void init_linefollower (motor_state_t* state) {
    *state = STATE_CONTROL_VELOCITY;
    int i;
    motion_parameter_unicycle_t unicycle;
    motor_parameter_t motor[NUM_MOTORS];
    motor_pid_t pid[NUM_MOTORS];
    /// Update parameter unicycle
    update_motion_parameter_unicycle(unicycle);
    // Update parameter motors
    for(i = 0; i < NUM_MOTORS; ++i) {
        update_motor_parameters(i, motor[i]);
        update_motor_pid(i, pid[i]);
    }
    
    IRsensor_Init();
}

motion_velocity_t loop_linefollower (motion_velocity_t* measure, motion_coordinate_t* coordinate) {
    // Our Hello World!!!! :)
    
    motion_velocity_t vel;
    vel.v = 0;
    vel.w = 0;
        
    return vel;
}

/*
 * Start a discharge of capacitor used to measure line reflectivity.
 * This functon put line as Output at "1" and exit, don't wait a full 
 * discharge
 * @param None
 * @param None
 */
void IRsensor_CapacitorDisCharge(void) {

    /*
     * Set IO0 ... IO6 pin as output
     */
    PORT_IO0 = OUTPUT; // Set IO0 as Output
    PORT_IO1 = OUTPUT; // Set IO1 as Output
    PORT_IO2 = OUTPUT; // Set IO2 as Output
    PORT_IO3 = OUTPUT; // Set IO3 as Output
    PORT_IO4 = OUTPUT; // Set IO4 as Output
    PORT_IO5 = OUTPUT; // Set IO5 as Output
    PORT_IO6 = OUTPUT; // Set IO6 as Output
    
    /* 
     * Setting pin to HighLevel discharge capacitor used in infrared sensor
     */ 
    PIN_IO0 = true;
    PIN_IO1 = true;
    PIN_IO2 = true;
    PIN_IO3 = true;
    PIN_IO4 = true;
    PIN_IO5 = true;
    PIN_IO6 = true;
    
}

/*
 * This function put line as Input and exit.
 * @param None
 * @param None
 */
void IRsensor_StartMeasure(void) {

    // Set IO0 ... IO6 pin as input
    PORT_IO0 = INPUT; // Set IO0 as Input
    PORT_IO1 = INPUT; // Set IO1 as Input
    PORT_IO2 = INPUT; // Set IO2 as Input
    PORT_IO3 = INPUT; // Set IO3 as Input
    PORT_IO4 = INPUT; // Set IO4 as Input
    PORT_IO5 = INPUT; // Set IO5 as Input
    PORT_IO6 = INPUT; // Set IO6 as Input
}



void IRsensor(void) {

    unsigned char i;
    signed char linedata;
    
    switch (line_sensor.fsm_state) {
        case 0:
            line_sensor.fsm_state = 1;
            discharge_timer=10;
            line_sensor.counter = 0;
            IRsensor_CapacitorDisCharge();
            for( i = 0; i<NUM_LINE_SENSOR; i++) {
                line_sensor.sensor_count[i]=0;
            }
            break;
                
        case 1:
            if (discharge_timer > 0) {
                discharge_timer--;
                IRsensor_CapacitorDisCharge();
            }
            else {
                line_sensor.fsm_state = 2;
            }
            break;
                
                
                
            case 2:
                IRsensor_StartMeasure(); 
                line_sensor.fsm_state = 3;
                break;
            case 3:
                //line_sensor.counter = 0;
                line_sensor.counter++;
                if(PIN_IO0) line_sensor.sensor_count[0]++;
                if(PIN_IO1) line_sensor.sensor_count[1]++;
                if(PIN_IO2) line_sensor.sensor_count[2]++;
                if(PIN_IO3) line_sensor.sensor_count[3]++;
                if(PIN_IO4) line_sensor.sensor_count[4]++;
                if(PIN_IO5) line_sensor.sensor_count[5]++;
                if(PIN_IO6) line_sensor.sensor_count[6]++;
                             
                if( line_sensor.counter == 16 )  line_sensor.fsm_state = 4;
                break;
            
            case 4:
//                // Counting Finish... convert measure in uSec.
                linedata = 0;
                for(i=0; i<NUM_LINE_SENSOR; i++) {
                    if(line_sensor.sensor_count[i] > 3) {     // TODO : 1 is a threshold, in future it must be calculated in a startup routine
                        linedata |= 1 << i;                   
                    }
                }
                // Now linedata contain a bit rappresentation of IR state                
                line_sensor.raw_line_value = linedata;
                line_sensor.line_position = line_conversion(linedata);
                
                line_sensor.fsm_state = 0;
                break;

            default:        // Beer condition :) 
                line_sensor.fsm_state = 0;
                break; 
    }

}


float line_conversion(int line) {
  int i;
  int value = 0, count = 0;
  for(i=0; i < BIT_LEN; ++i) {
    int mask = (1 << (i));
    if((line & (mask)) == (mask)) {
      value += i;
      count++;
    }
  }
  return value/count - BIT_LEN/2;
}




