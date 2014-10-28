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

    //add one byte for each char, two for each int, four for each float
    #define I2C_EEPROM_BUFF_SIZE_WRITE 7   //$$$$$$$$$$ define buffer size according to variables used
    #define I2C_EEPROM_BUFF_SIZE_READ 7    //$$$$$$$$$$ define buffer size according to variables used

    #define EEPROM_COMMAND 0xA0//EEPROM address. 0xA0 to 0xAE->8 pages x 256Byte

/******************************************************************************/
/* System Function Prototypes                                                 */
/******************************************************************************/
    
    typedef char boolean;
    typedef void (*I2C_callbackFunc)(boolean);

    void I2C_doneReadEEpromData(boolean I2CtrxOK);
    void I2C_doneWriteEEpromData(boolean I2CtrxOK);
    void I2C_readEEprom(boolean I2CtrxOK); // read from EEPROM
    void I2C_WriteEEprom(void); // write to EEPROM
    extern boolean I2C_Write(unsigned char command, unsigned char* pcommandData, unsigned char commandDataSize, unsigned char* ptxData, unsigned int txSize, I2C_callbackFunc pCallback);
    extern boolean I2C_Read(unsigned char command, unsigned char* pcommandData, unsigned char commandDataSize, unsigned char* prxData, unsigned int rxSize, I2C_callbackFunc pCallback);

#ifdef	__cplusplus
}
#endif

#endif	/* EEPROM_H */

