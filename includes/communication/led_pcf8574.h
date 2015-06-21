/*
 * Copyright (C) 2015 Officine Robotiche
 * Author: Mauro Soligo
 * email:  mauro.soligo@katodo.com
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

#ifndef LED_PCF8574_H
#define	LED_PCF8574_H

#ifdef	__cplusplus
extern "C" {
#endif

/******************************************************************************/
/* System Level #define Macros                                                */
/******************************************************************************/
#define PCF8574_LED1    1
#define PCF8574_LED2    2
#define PCF8574_LED3    4
#define PCF8574_LED4    8
#define PCF8574_LED5    16
#define PCF8574_LED6    32
#define PCF8574_LED7    64
    
    
/******************************************************************************/
/* System Function Prototypes                                                 */
/******************************************************************************/
   
     /**
     * 
     * @param led
     * @return 
     */
    bool PCF8574_LED_write(unsigned char led);
#ifdef	__cplusplus
}
#endif

#endif	/* EEPROM_H */

