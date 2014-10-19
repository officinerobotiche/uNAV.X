/* 
 * File:   I2c.h
 * Author: guiott
 *
 * Created on 21 settembre 2014, 18.42
 */

#ifndef I2C_H
#define	I2C_H

#ifdef	__cplusplus
extern "C" {
#endif

// Types
#define true	1
#define false	0
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


// <editor-fold defaultstate="collapsed" desc="Prototypes">

void I2C_init(void);

boolean I2C_Write(unsigned char command, unsigned char* pcommandData, unsigned char commandDataSize, unsigned char* ptxData, unsigned int txSize, I2C_callbackFunc pCallback);
boolean I2C_Read(unsigned char command, unsigned char* pcommandData, unsigned char commandDataSize, unsigned char* prxData, unsigned int rxSize, I2C_callbackFunc pCallback);

// Check for I2C ACK on command
boolean I2C_checkACK(unsigned int command, I2C_callbackFunc pCallback);

// Trigger the I2C service routine to run at low priority
void I2C_trigger_service(void);

// Trigger the I2C service routine to run at low priority in libUDB.c
void I2C_init(void);

// Reset the I2C module
void I2C_reset(void);

// Check if the I2CCON and I2CSTAT register are normal
boolean I2C_Normal(void);

void I2C_start(void) ;
void I2C_idle(void) ;
void I2C_doneRead(void);
void I2C_recstore(void);
void I2C_rerecen(void);
void I2C_recen(void);
void I2C_writeStop(void);
void I2C_stopRead(void);
void I2C_writeData(void);
void I2C_readCommand(void);
void I2C_writeCommand(void);
void I2C_startWrite(void);
void I2C_readStart(void);
void I2C_Failed(void);
void I2C_doneWrite(void);
void I2C_writeCommandData(void);
void serviceI2C(void);  // service the I2C
// </editor-fold>

#define I2C_SDA    _LATB9    //  from _RA3, _RA2, mods per Bill P.
#define I2C_SCL    _LATB8
#define _I2CEN  I2C1CONbits.I2CEN
#define	NULL	(0)

I2C_callbackFunc pI2C_callback = NULL;

int I2C_ERROR   = 0 ;
int I2CMAXS     = 0 ;
int I2CMAXQ    = 0 ;

// Port busy flag.  Set true until initialized
boolean I2C_Busy = true;

void (* I2C_state ) ( void ) = &I2C_idle ;

#define I2CBRGVAL ( (int)(((1/100e3) - 130E-9) * FCY)-2 ) // 392 // 100 Khz
// #define I2C2BRGVAL 60   //  **** WIP **** orig. def 60, 200 Khz  mod code [ per latest trunk revision]  **** WIP ****

#define I2C_NORMAL ( (I2CSTAT & 0b0000010011000000) == 0 )    // There is the queue, it's ok if the module is reading

#define I2C_QUEUE_DEPTH        3
#define INVALID_HANDLE 0xFFFF

I2Cqueue    i2c_queue[I2C_QUEUE_DEPTH];

unsigned int I2C_Index = 0; // index into the write buffer

unsigned char I2C_CommandByte  = 0;
unsigned int I2C_tx_data_size = 0;  // tx data size
unsigned int I2C_rx_data_size = 0;  // rx data size
unsigned int I2C_command_data_size = 0; // command data size

unsigned char* pI2CBuffer = NULL;   // pointer to buffer
unsigned char* pI2CcommandBuffer = NULL;    // pointer to receive  buffer

unsigned int I2C2_service_handle = INVALID_HANDLE;

unsigned int    I2C2_ERROR = 0;





#ifdef	__cplusplus
}
#endif

#endif	/* I2C_H */

