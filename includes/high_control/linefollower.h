/*
 * Copyright (C) 2015 Officine Robotiche
 * Author: Mauro Soligo
 * email:  mauro.soligo@officinerobotiche.it
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


#ifndef LINEFOLLOWER_H
#define	LINEFOLLOWER_H


#ifdef	__cplusplus
extern "C" {
#endif
    /**************************************************************************/
    /* System Level #define Macros                                            */
    /**************************************************************************/
    
    #define NUM_LINE_SENSOR     7
    
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
    

    /**************************************************************************/
    /* LineFollower structure                                                 */
    /**************************************************************************/


    /**
     * Data structure used to manage linefollower sensor
     * - fsm_state  : state of "Finited State Machine" : Charge, discharge, Measure...
     * - timebase : Configured during init, uSec time base if main function... used to calculate reading time.
     * - sensor_time[NUM_LINE_SENSOR] : Measure time of each sensor
     * 
     * - data: a single byte with a rapresentation of line respect at sensor.
     *          127 : Line on last sensor ( IR7 )
     *          0 : Line centered to sensor
     *          -127: Line on first sensor ( IR0 )
     * 
     */
    typedef struct linesensor {
        int8_t fsm_state;
        //int16_t timebase;
        //float sensor_time[NUM_LINE_SENSOR];
        
        int16_t counter;
        int16_t weight[NUM_LINE_SENSOR];    
        float position;
        int16_t sensor_count[NUM_LINE_SENSOR];
        int8_t data;
    } linesensor_t;
    #define LNG_LINESENSOR sizeof(linesensor_t)    
    
    /**************************************************************************/
    /* System Function Prototypes                                             */
    /**************************************************************************/
    
    void init_linefollower (motor_state_t* state);

    motion_velocity_t loop_linefollower (motion_velocity_t* measure, motion_coordinate_t* coordinate);
    
    /**************************************************************************/
    /* IR sensor reading related functions                                    */
    /**************************************************************************/    
    void IRsensor(void);
    
    /*
     * Start a discharge of capacitor used to measure line reflectivity.
     * This functon put line as Output at "1" and exit, don't wait a full 
     * discharge
     * @param None
     * @param None
     */
    void IRsensor_CapacitorDisCharge(void);  
    
     /*
     * This functon put line as Input and exit.
     * @param None
     * @param None
     */
    void IRsensor_StartMeasure(void);

    /**************************************************************************/
    /* Vatious                                    */
    /**************************************************************************/ 
    
    unsigned char GrayToDecimal_7bit(unsigned char gray);
    int LineCodeToDecimal_7bit(unsigned char line);
    
#ifdef	__cplusplus
}
#endif

#endif	/* LINEFOLLOWER_H */
