/* 
 * File:   linefollower.c
 * Author: Marco
 *
 * Created on 25 aprile 2015, 10.36
 */

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

#include "control/high_level_control.h"
#include "control/motors_PID.h"
#include "control/linefollower.h"

extern velocity_t vel_rif, vel_mis;
extern unsigned int counter_alive[NUM_MOTORS];

linesensor_t line_sensor;
//int16_t pippoooo;

void linefollowing()
{
    //Reset time emergency
    counter_alive[0] = 0;
    counter_alive[1] = 0;
               
    // Our Hello World!!!! :)
 
    if(line_sensor.sensor_time[0] != 0)
    {
        vel_rif.v = (float)line_sensor.sensor_time[0] / 10000; //0.0;
    }
    else
        vel_rif.v = 0.0;
    
    IRsensor();
    
//    /*
//     *  test of sequential calling of IRsensor_XXXXX function
//     *  to try a simple measure to evaluate sensor performance with 
//     *  Oscilloscope
//     */
//    if(pippoooo == 0)
//    {   pippoooo = 1;
//        IRsensor_CapacitorDisCharge();
//        
//    }
//    else
//    {   
//        if(pippoooo ==1 )
//        {   pippoooo++;
//            IRsensor_StartMeasure(); 
//        }
//        else
//        {
//            if( pippoooo < 16 )
//                pippoooo++;
//            else
//                pippoooo = 0;
//        }
//    }
}

void IRsensor_Init(void)
{   
    line_sensor.timebase = 1000;    // 1mSec = 1000 uSec
    
}

void IRsensor(void)
{
    unsigned char i;
       /*
     *  test of sequential calling of IRsensor_XXXXX function
     *  to try a simple measure to evaluate sensor performance with 
     *  Oscilloscope
     */
    
    
    switch (line_sensor.fsm_state) {
            case 0:
                line_sensor.fsm_state = 1;
                line_sensor.counter = 0;
                IRsensor_CapacitorDisCharge();
                for( i = 0; i<NUM_LINE_SENSOR; i++)
                {
                    line_sensor.sensor_time[i]=0;
                }
                break;
            case 1:
                IRsensor_StartMeasure(); 
                line_sensor.fsm_state = 2;
                break;
            case 2:
                //line_sensor.counter = 0;
                line_sensor.counter++;
                if(PIN_IO0) line_sensor.sensor_count[0]++;
                if(PIN_IO1) line_sensor.sensor_count[1]++;
                if(PIN_IO2) line_sensor.sensor_count[2]++;
                if(PIN_IO3) line_sensor.sensor_count[3]++;
                if(PIN_IO4) line_sensor.sensor_count[4]++;
                if(PIN_IO5) line_sensor.sensor_count[5]++;
                if(PIN_IO6) line_sensor.sensor_count[6]++;
                
                             
                if( line_sensor.counter == 16 )  line_sensor.fsm_state = 3;
                break;
            
            case 3:
                // Counting Finish... convert measure in uSec.
                for(i=0; i<NUM_LINE_SENSOR; i++)
                {
                    line_sensor.sensor_time[i] = line_sensor.sensor_count[i] * line_sensor.timebase;
                }

                line_sensor.fsm_state = 4;
                break;
                
            case 4:
                 line_sensor.fsm_state = 0;  
                break;
            default:        // Beer condition :) 
                line_sensor.fsm_state = 0;
                break; 
    }

}


void IRsensor_CapacitorDisCharge(void)
{   
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

void IRsensor_StartMeasure(void)
{
    // Set IO0 ... IO6 pin as input
    PORT_IO0 = INPUT; // Set IO0 as Input
    PORT_IO1 = INPUT; // Set IO1 as Input
    PORT_IO2 = INPUT; // Set IO2 as Input
    PORT_IO3 = INPUT; // Set IO3 as Input
    PORT_IO4 = INPUT; // Set IO4 as Input
    PORT_IO5 = INPUT; // Set IO5 as Input
    PORT_IO6 = INPUT; // Set IO6 as Input
}
