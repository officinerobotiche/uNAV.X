/*
 * Copyright (C) 2014 Officine Robotiche
 * Author: Guido Ottaviani
 * email:  guido@guiott.com
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

#ifndef EEPROM_H
#define	EEPROM_H

#ifdef	__cplusplus
extern "C" {
#endif

/******************************************************************************/
/* System Level #define Macros                                                */
/******************************************************************************/
    
    typedef void (*NVMemory_callbackFunc)(bool);

/******************************************************************************/
/* System Function Prototypes                                                 */
/******************************************************************************/
    
    /**
     * 
     * @param argc
     * @param argv
     */
    void EEPROM_service(int argc, char* argv);
    /**
     * 
     */
    void EEPROM_init(void);
    /**
     * 
     */
    void EEPROM_service_trigger(void);
    /**
     * 
     * @param eeprom_address
     * @param rdBuffer
     * @param address
     * @param rdSize
     * @param pCallback
     * @return 
     */
    bool EEPROM_read(uint8_t eeprom_address, uint8_t* rdBuffer, uint16_t address, uint16_t rdSize, NVMemory_callbackFunc pCallback);
    /**
     * 
     * @param eeprom_address
     * @param wrBuffer
     * @param address
     * @param wrSize
     * @param pCallback
     * @return 
     */
    bool EEPROM_write(uint8_t eeprom_address, uint8_t* wrBuffer, uint16_t address, uint16_t wrSize, NVMemory_callbackFunc pCallback);
    
#ifdef	__cplusplus
}
#endif

#endif	/* EEPROM_H */

