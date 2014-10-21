/* 
 * File:   eeprom.h
 * Author: guiott
 *
 * Created on 13 ottobre 2014, 14.43
 */

#ifndef EEPROM_H
#define	EEPROM_H

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

typedef char boolean;
typedef void (*I2C_callbackFunc)(boolean);

void I2C_doneReadEEpromData(boolean I2CtrxOK);
void I2C_doneWriteEEpromData(boolean I2CtrxOK);
void I2C_readEEprom(boolean I2CtrxOK);   // read from EEPROM
void I2C_WriteEEprom(void);              // write to EEPROM
extern boolean I2C_Write(unsigned char command, unsigned char* pcommandData, unsigned char commandDataSize, unsigned char* ptxData, unsigned int txSize, I2C_callbackFunc pCallback);
extern boolean I2C_Read(unsigned char command, unsigned char* pcommandData, unsigned char commandDataSize, unsigned char* prxData, unsigned int rxSize, I2C_callbackFunc pCallback);

//add one byte for each char, two for each int, four for each float
#define I2C_EEPROM_BUFF_SIZE_WRITE 7                    //$$$$$$$$$$ define buffer size according to variables used
#define I2C_EEPROM_BUFF_SIZE_READ 7                     //$$$$$$$$$$ define buffer size according to variables used

#define EEPROM_COMMAND 0X24                             //$$$$$$$$$$ define EEPROM address

#endif	/* EEPROM_H */

