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
 * 
 * Original code:
 * https://code.google.com/p/gentlenav/source/browse/trunk/libUDB/24LC256.c
 */

/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

#include <system/events.h>

#include <peripherals/i2c.h>

#include "communication/eeprom.h"

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

#define PCF8574_LED_COMMAND  0x40

uint8_t wrBuffer[3] = {0, 7, 8};
//uint8_t wrBuffer1[3] = {2, 3, 4};
bool pcf8574_led_not_busy = true;

static void PCF8574_LED_callback(bool callback_status);



bool PCF8574_LED_write(unsigned char led) {
    bool status;
    
    wrBuffer[0] = ~led;

    if(pcf8574_led_not_busy) {
        status = I2C_Write(PCF8574_LED_COMMAND, wrBuffer, 1, &PCF8574_LED_callback);
        pcf8574_led_not_busy = false;
    }
    return true;
}

static void PCF8574_LED_callback(bool callback_status) {
    pcf8574_led_not_busy = callback_status;

}

