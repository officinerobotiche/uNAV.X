#include "eeprom.h"

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

/**
 * Write data to EEprom
 */
void I2C_WriteEEprom(void)
{
    I2CEEpromWriteBuff.I.EeepromA = 1;   //$$$$$$$$$$ example
    I2CEEpromWriteBuff.I.EeepromB = 1;   //$$$$$$$$$$ example
    I2CEEpromWriteBuff.I.EeepromC = 1;   //$$$$$$$$$$ example
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
