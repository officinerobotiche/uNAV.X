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

#define EEPROM "EEPROM"
static string_data_t _MODULE_EEPROM = {EEPROM, sizeof (EEPROM)};

typedef enum _EEPROM_STATES {
    MCP24LC256_STATE_STOPPED,
    MCP24LC256_STATE_READING,
    MCP24LC256_STATE_WRITING,
    MCP24LC256_STATE_WAITING_WRITE,
    MCP24LC256_STATE_FAILED_TRX,
} EEPROM_STATES_T;

typedef union _ee_address {
    uint8_t commandData[2];
    uint16_t address;
} ee_addres_t;
#define LNG_EEPROM_ADDRESS sizeof(ee_addres_t)

static uint8_t eeprom_address = 0;

static EEPROM_STATES_T memory_state = MCP24LC256_STATE_STOPPED;
static hEvent_t memory_service_handle = INVALID_HANDLE;

static NVMemory_callbackFunc pcallerCallback = NULL;

static uint16_t MCP24LC256_write_address;
static uint16_t MCP24LC256_write_size;
static ee_addres_t ee_address;
static uint8_t* MCP24LC256_pwrBuffer = NULL;

static bool EEPROM_write_chunk(void);
static void EEPROM_callback(bool I2CtrxOK);

/*****************************************************************************/
/* EEPROM Functions                                                          */
/*****************************************************************************/

void EEPROM_service(int argc, char* argv) {
    switch (memory_state) {
        case MCP24LC256_STATE_WAITING_WRITE:
            I2C_checkACK(MCP24LC256_COMMAND, &EEPROM_callback);
            break;
        case MCP24LC256_STATE_FAILED_TRX:
            I2C_checkACK(MCP24LC256_COMMAND, &EEPROM_callback);
            break;
        default:
            break;
    }
}

void EEPROM_init(void) {
    memory_service_handle = register_event_p(&EEPROM_service, &_MODULE_EEPROM, EVENT_PRIORITY_LOW);
}

void EEPROM_service_trigger(void) {
    trigger_event(memory_service_handle);
}

bool EEPROM_read(uint8_t eeprom_addr, uint8_t* rdBuffer, uint16_t address, uint16_t rdSize, NVMemory_callbackFunc pCallback) {
    if (memory_state != MCP24LC256_STATE_STOPPED) return false;
    memory_state = MCP24LC256_STATE_READING;
    
    ee_address.address = address;

    pcallerCallback = pCallback;
    // Get first 3 byte
    eeprom_addr &= 0x7;
    if (I2C_Read(MCP24LC256_COMMAND | eeprom_addr, ee_address.commandData, LNG_EEPROM_ADDRESS, rdBuffer, rdSize, &EEPROM_callback) == false) {
        memory_state = MCP24LC256_STATE_STOPPED;
        return false;
    }
    return true;
}

bool EEPROM_write(uint8_t eeprom_addr, uint8_t* wrBuffer, uint16_t address, uint16_t wrSize, NVMemory_callbackFunc pCallback) {
    if (memory_state != MCP24LC256_STATE_STOPPED) return false;

    // Get first 3 byte
    eeprom_address = eeprom_addr & 0x7;
    // Check address range.
    if (address > 0x7FFF) return false;

    MCP24LC256_pwrBuffer = wrBuffer;

    MCP24LC256_write_address = address;
    MCP24LC256_write_size = wrSize;

    pcallerCallback = pCallback;

    return EEPROM_write_chunk();
}

static bool EEPROM_write_chunk(void) {
    uint16_t writeSize = MCP24LC256_write_size;
    // Truncate write at page boundary
    if (writeSize > 0x40)
        writeSize = 0x40;

    // Check if writes are finished
    if (writeSize == 0) {
        memory_state = MCP24LC256_STATE_STOPPED;
        if (pcallerCallback != NULL) pcallerCallback(true);
        pcallerCallback = NULL;
        return true;
    }

    // Find remaining bytes in the page
    uint16_t pageRemainaing = 0x40 - (MCP24LC256_write_address & 0x3F);

    if (writeSize > pageRemainaing) writeSize = pageRemainaing;

    ee_address.address = MCP24LC256_write_address;

    memory_state = MCP24LC256_STATE_WRITING;

    if (I2C_Write(MCP24LC256_COMMAND | eeprom_address, ee_address.commandData, LNG_EEPROM_ADDRESS, MCP24LC256_pwrBuffer, writeSize, &EEPROM_callback) == false) {
        memory_state = MCP24LC256_STATE_FAILED_TRX;
        return false;
    }

    MCP24LC256_write_size -= writeSize;
    MCP24LC256_write_address += writeSize;
    MCP24LC256_pwrBuffer = &MCP24LC256_pwrBuffer[writeSize];
    return true;
}

static void EEPROM_callback(bool I2CtrxOK) {
    if (I2CtrxOK == false) {
        // If waiting for write ACK, continue to wait
        if (memory_state != MCP24LC256_STATE_WAITING_WRITE) {
            memory_state = MCP24LC256_STATE_FAILED_TRX;
            if (pcallerCallback != NULL) pcallerCallback(true);
            pcallerCallback = NULL;
        }
        return;
    }

    switch (memory_state) {
        case MCP24LC256_STATE_READING:
            if (pcallerCallback != NULL) pcallerCallback(true);
            pcallerCallback = NULL;
            memory_state = MCP24LC256_STATE_STOPPED;
            break;
        case MCP24LC256_STATE_WRITING:
            memory_state = MCP24LC256_STATE_WAITING_WRITE;
            if (MCP24LC256_write_size == 0) {
                EEPROM_write_chunk();
            }
            break;
        case MCP24LC256_STATE_WAITING_WRITE:
            EEPROM_write_chunk();
            break;
        case MCP24LC256_STATE_FAILED_TRX:
            memory_state = MCP24LC256_STATE_STOPPED;
            break;
        default:
            memory_state = MCP24LC256_STATE_FAILED_TRX;
            break;
    }
}
