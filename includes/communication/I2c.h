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

#ifndef I2C_H
#define	I2C_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "system/system.h"
    
/******************************************************************************/
/* System Level #define Macros                                                */
/******************************************************************************/
    
    #define I2C_SDA    _LATB9    //  from _RA3, _RA2, mods per Bill P.
    #define I2C_SCL    _LATB8
    #define _I2CEN  I2C1CONbits.I2CEN
    #define	NULL	(0)

    #define I2CBRGVAL ( (int)(((1/100e3) - 130E-9) * FCY)-2 ) // 392 // 100 Khz
    // #define I2C2BRGVAL 60   //  **** WIP **** orig. def 60, 200 Khz  mod code [ per latest trunk revision]  **** WIP ****

    #define I2C_NORMAL ( (I2CSTAT & 0b0000010011000000) == 0 )    // There is the queue, it's ok if the module is reading

    #define I2C_QUEUE_DEPTH        3
    #define INVALID_HANDLE 0xFFFF


    typedef char boolean;

    // callback type for I2C user
    typedef void (*I2C_callbackFunc)(boolean);

    typedef struct tag_I2Cqueue
    {
            boolean pending;
            boolean rW;
            unsigned char command;
            unsigned char* pcommandData;
            unsigned char commandDataSize;
            unsigned char* pData;
            unsigned int Size;
            I2C_callbackFunc pCallback;
    } I2Cqueue;

/******************************************************************************/
/* System Function Prototypes                                                 */
/******************************************************************************/

    /**
     * Trigger the I2C service routine to run at low priority in libUDB.c
     * initialize the I2C peripheral
     */
    void InitI2C(void);
    /**
     * Reset the I2C module
     */
    void I2C_reset(void);
    /**
     * 
     * @param command
     * @param pcommandData
     * @param commandDataSize
     * @param ptxData
     * @param txSize
     * @param pCallback
     * @return 
     */
    bool I2C_Write(unsigned char command, unsigned char* pcommandData, unsigned char commandDataSize, unsigned char* ptxData, unsigned int txSize, I2C_callbackFunc pCallback);
    /**
     * 
     * @param command
     * @param pcommandData
     * @param commandDataSize
     * @param prxData
     * @param rxSize
     * @param pCallback
     * @return 
     */
    bool I2C_Read(unsigned char command, unsigned char* pcommandData, unsigned char commandDataSize, unsigned char* prxData, unsigned int rxSize, I2C_callbackFunc pCallback);
    /**
     * 
     * @return 
     */
    bool I2C_serve_queue(void);
    /**
     * Check if I2C port is available for use.
     * @return 
     */
    inline bool I2C_CheckAvailable(void);
    /**
     * 
     */
    void I2C_startWrite(void);
    /**
     * Write command byte without checking ACK first
     */
    void I2C_writeCommand(void);
    /**
     *  Write command data (address or similar)
     */
    void I2C_writeCommandData(void);
    
    /* READ FUNCTIONS */
    
    /**
     * Start a read after a write by setting the start bit again
     */
    void I2C_readStart(void);
    /**
     * Send the command to read
     */
    void I2C_readCommand(void);
    /**
     * Check for ACK.  If ok, start receive mode, otherwise abandon.
     */
    void I2C_recen(void);
    /**
     * 
     */
    void I2C_recstore(void);
    /**
     * 
     */
    void I2C_stopRead(void);
    /**
     * 
     */
    void I2C_rerecen(void);
    /**
     * 
     */
    void I2C_doneRead(void);
    
    /* WRITE FUNCTIONS */
    
    /**
     * 
     */
    void I2C_writeData(void);   
    /**
     * Stop a write
     */
    void I2C_writeStop(void);
    /**
     * 
     */
    void I2C_doneWrite(void);
    
    /* SERVICE FUNCTIONS */
    
    /**
     *  do nothing
     */
    void I2C_idle(void);
    /**
     * On failure, stop the bus, go into idle and callback with failure
     */
    void I2C_Failed(void);

    /************* ??????????????????????? *************/
    
    // Trigger the I2C service routine to run at low priority
    void I2C_trigger_service(void);

    void I2C_start(void) ;
    
    // Check if the I2CCON and I2CSTAT register are normal
    bool I2C_Normal(void);
    
    // Check for I2C ACK on command
    bool I2C_checkACK(unsigned int command, I2C_callbackFunc pCallback);
    
    /************* ??????????????????????? *************/

#ifdef	__cplusplus
}
#endif

#endif	/* I2C_H */

