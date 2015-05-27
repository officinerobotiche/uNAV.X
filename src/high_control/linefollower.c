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

#include "high_control/manager.h"

/* Device header file */
/*
#if defined(__XC16__)
#include <xc.h>
#elif defined(__C30__)
#if defined(__dsPIC33E__)
#include <p33Exxxx.h>
#elif defined(__dsPIC33F__)
#include <p33Fxxxx.h>
#endif
#endif


#include "control/high_level_control.h"
#include "control/motors_PID.h"
#include "control/linefollower.h"
 */

//#include <stdint.h>        /* Includes uint16_t definition                    */
//#include <stdbool.h>       /* Includes true/false definition                  */

/*****************************************************************************/
/* Global Variable Declaration                                               */
/*****************************************************************************/

motor_t linefollower_test;

linesensor_t line_sensor;

/*****************************************************************************/
/* User Functions                                                            */
/*****************************************************************************/

void IRsensor_Init(void) {
   
    //line_sensor.timebase = 1000;    // 1mSec = 1000 uSec
    
    line_sensor.weight[0] = 8;
    line_sensor.weight[1] = 4;
    line_sensor.weight[2] = 2;
    line_sensor.weight[3] = 1;
    line_sensor.weight[4] = -1;
    line_sensor.weight[5] = -2;
    line_sensor.weight[6] = -4;
    line_sensor.weight[7] = -8;
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

void linefollowing() {
       

 
//    linefollower_test.current = 4;
//    linefollower_test.refer_vel = 2500;
    
//    if(line_sensor.sensor_time[0] != 0) {
//    
//        vel_rif.v = (float)line_sensor.sensor_time[0] / 10000; //0.0;
//    }
//    else {
//        vel_rif.v = 0.0;
//    }
    //IRsensor();
}

void IRsensor(void) {

    unsigned char i;
    signed char linedata;
    
    switch (line_sensor.fsm_state) {
            case 0:
                line_sensor.fsm_state = 1;
                line_sensor.counter = 0;
                IRsensor_CapacitorDisCharge();
                for( i = 0; i<NUM_LINE_SENSOR; i++) {
                    line_sensor.sensor_count[i]=0;
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
//                // Counting Finish... convert measure in uSec.
                line_sensor.position = 0;
                linedata = 0;
                for(i=0; i<NUM_LINE_SENSOR; i++) {   
                    // Convert measure in uSec.
                    //line_sensor.sensor_time[i] = line_sensor.sensor_count[i] * line_sensor.timebase;
                    if(line_sensor.sensor_count[i] > 1) {     // TODO : 1 is a threshold, in future it must be calculated in a startup routine
                        linedata |= 1 << i;                   
                    }
                }
                // Now linedata contain a bit rappresentation of IR state
                linefollower_test.current =   linedata * 1000;
                linedata = LineCodeToDecimal_7bit(linedata);
                 // Now LineData contain a decimal rappresentation of line respect to sensor
                 // 0 = centered
                linefollower_test.refer_vel = linedata * 1000;
              
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

unsigned char GrayToDecimal_7bit(unsigned char gray) {

    // simpler Gray to decimal conversion
    
    switch(gray) {
        
        /*     
         *     IR7 to IR0 bit
         * 
         *     IIIIIIIII
         *     RRRRRRRRR     
         *     X76543210
         */
        
        /*
         * Only data with all bit to 0/1 and data with 1 or 2 contiguous bit to "1" can be considered conidered UsefullData.
         * This value is flagged  in this case
         */
        case 0b0000000 :   return 0;    break; /* UD */
        case 0b0000001 :   return 1;    break; /* UD */
        case 0b0000011 :   return 2;    break; /* UD */
        case 0b0000010 :   return 3;    break; /* UD */
        case 0b0000110 :   return 4;    break; /* UD */
        case 0b0000111 :   return 5;    break;
        case 0b0000101 :   return 6;    break;
        case 0b0000100 :   return 7;    break; /* UD */
        case 0b0001100 :   return 8;    break; /* UD */
        case 0b0001101 :   return 9;    break;
        case 0b0001111 :   return 10;   break;
        case 0b0001110 :   return 11;   break;
        case 0b0001010 :   return 12;   break;
        case 0b0001011 :   return 13;   break;
        case 0b0001001 :   return 14;   break;
        case 0b0001000 :   return 15;   break; /* UD */
        case 0b0011000 :   return 16;   break; /* UD */
        case 0b0011001 :   return 17;   break;
        case 0b0011011 :   return 18;   break;
        case 0b0011010 :   return 19;   break;
        case 0b0011110 :   return 20;   break;
        case 0b0011111 :   return 21;   break;
        case 0b0011101 :   return 22;   break;
        case 0b0011100 :   return 23;   break;
        case 0b0010100 :   return 24;   break;
        case 0b0010101 :   return 25;   break;
        case 0b0010111 :   return 26;   break;
        case 0b0010110 :   return 27;   break;
        case 0b0010010 :   return 28;   break;
        case 0b0010011 :   return 29;   break;
        case 0b0010001 :   return 30;   break;    
        case 0b0010000 :   return 31;   break;    
        case 0b0110000 :   return 32;   break; /* UD */
        case 0b0110001 :   return 33;   break;    
        case 0b0110011 :   return 34;   break;    
        case 0b0110010 :   return 35;   break;    
        case 0b0110110 :   return 36;   break;    
        case 0b0110111 :   return 37;   break;    
        case 0b0110101 :   return 38;   break;    
        case 0b0110100 :   return 39;   break;    
        case 0b0111100 :   return 40;   break;    
        case 0b0111101 :   return 41;   break;    
        case 0b0111111 :   return 42;   break;    
        case 0b0111110 :   return 43;   break;        
        case 0b0111010 :   return 44;   break;        
        case 0b0111011 :   return 45;   break;        
        case 0b0111001 :   return 46;   break;        
        case 0b0111000 :   return 47;   break;        
        case 0b0101000 :   return 48;   break;        
        case 0b0101001 :   return 49;   break;        
        case 0b0101011 :   return 50;   break;
        case 0b0101010 :   return 51;   break;
        case 0b0101110 :   return 52;   break;
        case 0b0101111 :   return 53;   break;
        case 0b0101101 :   return 54;   break;
        case 0b0101100 :   return 55;   break;
        case 0b0100100 :   return 56;   break;
        case 0b0100101 :   return 57;   break;
        case 0b0100111 :   return 58;   break;
        case 0b0100110 :   return 59;   break;
        case 0b0100010 :   return 60;   break;
        case 0b0100011 :   return 61;   break;
        case 0b0100001 :   return 62;   break;
        case 0b0100000 :   return 63;   break; /* UD */
        case 0b1100000 :   return 64;   break; /* UD */
        
        case 0b1100001 :   return 65;   break;
        case 0b1100011 :   return 66;   break;
        case 0b1100010 :   return 67;   break;
        case 0b1100110 :   return 68;   break;
        case 0b1100111 :   return 69;   break;
        case 0b1100101 :   return 70;   break;
        case 0b1100100 :   return 71;   break;
        case 0b1101100 :   return 71;   break;
        case 0b1101101 :   return 73;   break;
        case 0b1101111 :   return 74;   break;
        case 0b1101110 :   return 75;   break;
        case 0b1101010 :   return 76;   break;
        case 0b1101011 :   return 77;   break;
        case 0b1101001 :   return 78;   break;
        case 0b1101000 :   return 79;   break;
        case 0b1111000 :   return 80;   break;
        case 0b1111001 :   return 81;   break;
        case 0b1111011 :   return 82;   break;
        case 0b1111010 :   return 83;   break;
        case 0b1111110 :   return 84;   break;
        case 0b1111111 :   return 85;   break;
        case 0b1111101 :   return 86;   break;
        case 0b1111100 :   return 87;   break;
        case 0b1110100 :   return 88;   break;
        case 0b1110101 :   return 89;   break;
        case 0b1110111 :   return 90;   break;
        case 0b1110110 :   return 91;   break;
        case 0b1110010 :   return 92;   break;
        case 0b1110011 :   return 93;   break;
        case 0b1110001 :   return 94;   break;
        case 0b1110000 :   return 95;   break;
        case 0b1010000 :   return 96;   break;
        case 0b1010001 :   return 97;   break;
        case 0b1010011 :   return 98;   break;
        case 0b1010010 :   return 99;   break;
        case 0b1010110 :   return 100;  break;
        case 0b1010111 :   return 101;  break;
        case 0b1010101 :   return 102;  break;
        case 0b1010100 :   return 103;  break;
        case 0b1011100 :   return 104;  break;
        case 0b1011101 :   return 105;  break; 
        case 0b1011111 :   return 106;  break;
        case 0b1011110 :   return 107;  break;
        case 0b1011010 :   return 108;  break; 
        case 0b1011011 :   return 109;  break; /* ok */
        case 0b1011001 :   return 110;  break;   
        case 0b1011000 :   return 111;  break; /* ok */  
        case 0b1001000 :   return 112;  break;
        case 0b1001001 :   return 113;  break;
        case 0b1001011 :   return 114;  break; 
        case 0b1001010 :   return 115;  break;
        case 0b1001110 :   return 116;  break; 
        case 0b1001111 :   return 117;  break;
        case 0b1001101 :   return 118;  break;
        case 0b1001100 :   return 119;  break; 
        case 0b1000100 :   return 120;  break;   
        case 0b1000101 :   return 121;  break;  
        case 0b1000111 :   return 122;  break;
        case 0b1000110 :   return 123;  break;
        case 0b1000010 :   return 124;  break;
        case 0b1000011 :   return 125;  break;
        case 0b1000001 :   return 126;  break;      
        case 0b1000000 :   return 127;  break;
        default : return 0;
    }
}


int LineCodeToDecimal_7bit(unsigned char line) {
    /*
    * Only data with all bit to 0/1 and data with 1 or 2 contiguous bit to "1" can be considered conidered UsefullData.
    * This value is flagged  in this case
    */
    
    
    switch(line) {
        
        /*     
         *     IR7 to IR0 bit
         * 
         *     IIIIIIIII
         *     RRRRRRRRR     
         *     X76543210
         */
        
        case 0b0000000 :   return 7;    break; /* Robot out of line */
        case 0b0000001 :   return 6;    break;
        case 0b0000011 :   return 5;    break;
        case 0b0000010 :   return 4;    break;
        case 0b0000110 :   return 3;    break;
        case 0b0000100 :   return 2;    break;
        case 0b0001100 :   return 1;    break;
        case 0b0001000 :   return 0;    break;  /* Line on center of sensor */
        case 0b0011000 :   return -1;    break;
        case 0b0010000 :   return -2;    break;
        case 0b0110000 :   return -3;    break;
        case 0b0100000 :   return -4;    break;
        case 0b1100000 :   return -5;    break;
        case 0b1000000 :   return -6;    break;
        case 0b1111111 :   return -7;    break; /* Robot on Start/Stop line or error */
        default :   return -8;
    }
}






//
//    #include<stdio.h> 
//    #include<conio.h> 
//
//    void main() 
//    {
//        int a[10],i=0,c=0,n;
//        printf("\n enter the gray code");
//        scanf("%d",&n);
//        while(n!=0)
//            {
//                a[i]=n%10;
//                n/=10;
//                i++;
//                c++;
//            }
//
//        for(i=c-1;i>=0;i--)
//        {
//            if(a[i]==1)
//            { 
//                if(a[i-1]==1)
//                    a[i-1]=0;
//                else
//                    a[i-1]=1;
//            }
//        } 
//        printf("\n the binary code is"); 
//        for(i=c-1;i>=0;i--) printf("%d",a[i]); 
//        getch(); 
//    } 