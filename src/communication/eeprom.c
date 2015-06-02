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
 * 
 * Original code:
 * https://code.google.com/p/gentlenav/source/browse/trunk/libUDB/24LC256.c
*/

/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

#include <system/events.h>

#include "communication/I2c.h"

#include "communication/eeprom.h"

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

#define MCP24LC256_COMMAND  0xA0
typedef void (*NVMemory_callbackFunc)(boolean);

enum MCP24LC256_STATES
{
        MCP24LC256_STATE_STOPPED,
        MCP24LC256_STATE_READING,
        MCP24LC256_STATE_WRITING,
        MCP24LC256_STATE_WAITING_WRITE,
        MCP24LC256_STATE_FAILED_TRX,
};

static uint16_t MCP24LC256_state = MCP24LC256_STATE_STOPPED;
static hEvent_t nv_memory_service_handle = INVALID_HANDLE;

static uint16_t MCP24LC256_Timer = 0;
static NVMemory_callbackFunc pcallerCallback = NULL;

static uint16_t MCP24LC256_write_address;
static uint16_t MCP24LC256_write_size;
static uint8_t commandData[4] = {0x00, 0x00}; 
static uint8_t* MCP24LC256_pwrBuffer = NULL;

/*********/
// TX Buffer
struct _WriteBuff
{
    char EeepromA;   //$$$$$$$$$$ just as an example of char variable
    int EeepromB;    //$$$$$$$$$$ just as an example of int variable
    float EeepromC;  //$$$$$$$$$$ just as an example of float variable
};

union __WriteBuff
{
    struct _WriteBuff I;// to use as integers or chars, little endian LSB first
    unsigned char C[I2C_EEPROM_BUFF_SIZE_WRITE];//to use as bytes to send on I2C buffer
}I2CEEpromWriteBuff;


// RX Buffer
struct _ReadBuff
{
    char EeepromD;   //$$$$$$$$$$ just as an example of char variable
    int EeepromE;    //$$$$$$$$$$ just as an example of int variable
    float EeepromF;  //$$$$$$$$$$ just as an example of float variable
};

union __ReadBuff
{
    struct _ReadBuff I;// to use as integers or chars, little endian LSB first
    unsigned char C[I2C_EEPROM_BUFF_SIZE_READ];//to use as bytes to send on I2C buffer
}I2CEEpromReadBuff;

unsigned char EEpromReadIndex[] = {0x00} ;    // Address of the first register to read
unsigned char EEpromWriteIndex[] = {0x00} ;   // Address of the first register to read

/******************************************************************************/
/* EEPROM Functions                                                           */
/******************************************************************************/

void nv_memory_service(void)
{
        switch (MCP24LC256_state)
        {
                case MCP24LC256_STATE_WAITING_WRITE:
                        I2C_checkACK(MCP24LC256_COMMAND, &MCP24LC256_callback);
                        break;
                case MCP24LC256_STATE_FAILED_TRX:
                        I2C_checkACK(MCP24LC256_COMMAND, &MCP24LC256_callback);
                        break;
        }
}

void nv_memory_init(void)
{
        nv_memory_service_handle = register_event(&nv_memory_service);
}

void nv_memory_service_trigger(void)
{
        trigger_event(nv_memory_service_handle);
}

boolean udb_nv_memory_read(uint8_t* rdBuffer, uint16_t address, uint16_t rdSize, NVMemory_callbackFunc pCallback)
{
        if (MCP24LC256_state != MCP24LC256_STATE_STOPPED) return false;
        MCP24LC256_state = MCP24LC256_STATE_READING;

        commandData[1] = (uint8_t) (address & 0xFF);
        commandData[0] = (uint8_t) ((address >> 8) & 0xFF);

        pcallerCallback = pCallback;

        if (I2C_Read(MCP24LC256_COMMAND, commandData, 2, rdBuffer, rdSize, &MCP24LC256_callback, 0) == false)
        {
                MCP24LC256_state = MCP24LC256_STATE_STOPPED;
                return false;
        }
        return true;
}

boolean udb_nv_memory_write(uint8_t* wrBuffer, uint16_t address, uint16_t wrSize, NVMemory_callbackFunc pCallback)
{
        if (MCP24LC256_state != MCP24LC256_STATE_STOPPED) return false;

        // Check address range.
        if (address > 0x7FFF) return false;
        if (address > 0x7FFF) return false;

        MCP24LC256_pwrBuffer = wrBuffer;

        MCP24LC256_write_address = address;
        MCP24LC256_write_size = wrSize;

        pcallerCallback = pCallback;

        return MCP24LC256_write_chunk();
}

static boolean MCP24LC256_write_chunk(void)
{
        uint16_t writeSize = MCP24LC256_write_size;
        // Truncate write at page boundary
        if (writeSize > 0x40)
                writeSize = 0x40;

        // Check if writes are finished
        if (writeSize == 0)
        {
                MCP24LC256_state = MCP24LC256_STATE_STOPPED;
                if (pcallerCallback != NULL) pcallerCallback(true);
                pcallerCallback = NULL;
                return true;
        }

        // Find remaining bytes in the page
        uint16_t pageRemainaing = 0x40 - (MCP24LC256_write_address & 0x3F);

        if (writeSize > pageRemainaing) writeSize = pageRemainaing;

        commandData[1] = (uint8_t)(MCP24LC256_write_address & 0xFF);
        commandData[0] = (uint8_t)((MCP24LC256_write_address >> 8) & 0xFF);

        MCP24LC256_state = MCP24LC256_STATE_WRITING;

        if (I2C_Write(MCP24LC256_COMMAND, commandData, 2, MCP24LC256_pwrBuffer, writeSize, &MCP24LC256_callback) == false)
        {
                MCP24LC256_Timer = 0;
                MCP24LC256_state = MCP24LC256_STATE_FAILED_TRX;
                return false;
        }

        MCP24LC256_write_size -= writeSize;
        MCP24LC256_write_address += writeSize;
        MCP24LC256_pwrBuffer = &MCP24LC256_pwrBuffer[writeSize];
        return true;
}

static void MCP24LC256_callback(boolean I2CtrxOK)
{
        if (I2CtrxOK == false)
        {
                MCP24LC256_Timer = 0;
                // If waiting for write ACK, continue to wait
                if (MCP24LC256_state != MCP24LC256_STATE_WAITING_WRITE)
                {
                        MCP24LC256_state = MCP24LC256_STATE_FAILED_TRX;
                        if (pcallerCallback != NULL) pcallerCallback(true);
                        pcallerCallback = NULL;
                }
                return;
        }

        switch (MCP24LC256_state)
        {
                case MCP24LC256_STATE_READING:
                        if (pcallerCallback != NULL) pcallerCallback(true);
                        pcallerCallback = NULL;
                        MCP24LC256_state = MCP24LC256_STATE_STOPPED;
                        break;
                case MCP24LC256_STATE_WRITING:
                        MCP24LC256_Timer = 0;
                        MCP24LC256_state = MCP24LC256_STATE_WAITING_WRITE;
                        break;
                case MCP24LC256_STATE_WAITING_WRITE:
                        MCP24LC256_write_chunk();
                        break;
                case MCP24LC256_STATE_FAILED_TRX:
                        MCP24LC256_state = MCP24LC256_STATE_STOPPED;
                        break;
                default:
                        MCP24LC256_state = MCP24LC256_STATE_FAILED_TRX;
                        break;
        }
}


/**
 * Write data to EEprom
 */
void I2C_WriteEEprom(void)
{
    I2CEEpromWriteBuff.I.EeepromA = 0xFF;   //$$$$$$$$$$ example
    I2CEEpromWriteBuff.I.EeepromB = 0xFFFF; //$$$$$$$$$$ example
    I2CEEpromWriteBuff.I.EeepromC = 1234,56789;   //$$$$$$$$$$ example
    I2C_Write(EEPROM_COMMAND, EEpromWriteIndex, 1, I2CEEpromWriteBuff.C, I2C_EEPROM_BUFF_SIZE_WRITE, &I2C_doneWriteEEpromData);
}

/**
 * Read data from EEPROM
 * @param I2CtrxOK
 */
void I2C_readEEprom(boolean I2CtrxOK)
{
    I2C_Read(EEPROM_COMMAND, EEpromReadIndex, 1, I2CEEpromReadBuff.C, I2C_EEPROM_BUFF_SIZE_READ, &I2C_doneReadEEpromData);
}

void I2C_doneReadEEpromData( boolean I2CtrxOK )
{
    return ;
}

void I2C_doneWriteEEpromData( boolean I2CtrxOK )
{
    return ;
}
