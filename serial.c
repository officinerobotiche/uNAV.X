/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

/* Device header file */
#if defined(__XC16__)
#include <xc.h>
#elif defined(__C30__)
#if defined(__dsPIC33E__)
#include <p33Exxxx.h>
#elif defined(__dsPIC33F__)
#include <p33Fxxxx.h>
#endif
#endif

#include <stdint.h>        /* Includes uint16_t definition   */
#include <stdbool.h>       /* Includes true/false definition */
#include <string.h>

#include "serial.h"

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

int (*pkg_parse) (unsigned char inchar) = &pkg_header;
unsigned char BufferTx[MAX_TX_BUFF] __attribute__((space(dma)));
packet_t receive_pkg;
char receive_header;
unsigned int index_data = 0;
error_pkg_t serial_error;

/******************************************************************************/
/* Comunication Functions                                                     */
/******************************************************************************/

/* Data structure:
 * ------------------------------------------------
 * | HEADER | LENGTH |       DATA           | CKS |
 * ------------------------------------------------
 *     1        2             3 -> n          n+1
 */

void init_buff_serial_error(){
    memset(serial_error.number, 0, BUFF_SERIAL_ERROR);
}

int decode_pkgs(unsigned char rxchar) {
    return (*pkg_parse)(rxchar);
}

int pkg_header(unsigned char rxchar) {
    if ((rxchar == HEADER_SYNC) || (rxchar == HEADER_ASYNC)) {
        receive_header = rxchar;
        pkg_parse = &pkg_length;
        return false;
    } else {
        return pkg_error(ERROR_HEADER);
    }
}

int pkg_length(unsigned char rxchar) {
    if (rxchar > MAX_RX_BUFF) {
        return pkg_error(ERROR_LENGTH);
    } else {
        pkg_parse = &pkg_data;
        receive_pkg.length = rxchar;
        return false;
    }
}

int pkg_data(unsigned char rxchar) {
    int cks_clc;
    if ((index_data + 1) == (receive_pkg.length + 1)) {
        pkg_parse = &pkg_header; //Restart parse serial packet
        if ((cks_clc = pkg_checksum(receive_pkg.buffer, 0, index_data)) == rxchar) { //checksum data
            index_data = 0; //flush index array data buffer
            return true;
        } else {
            bool t = pkg_error(ERROR_CKS);
            return t;
        }
    } else {
        receive_pkg.buffer[index_data] = rxchar;
        index_data++;
        return false;
    }
}

int pkg_error(int error) {
    index_data = 0;
    pkg_parse = &pkg_header; //Restart parse serial packet
    serial_error.number[(-error - 1)] += 1;
    return error;
}

unsigned char pkg_checksum(volatile unsigned char* Buffer, int FirstIndx, int LastIndx) {
    unsigned char ChkSum = 0;
    int ChkCnt;
    for (ChkCnt = FirstIndx; ChkCnt < LastIndx; ChkCnt++) {
        ChkSum += Buffer[ChkCnt];
    }
    return ChkSum;
}

/**
 * Write a string to the serial device.
 * \param s string to write
 * \throws boost::system::system_error on failure
 */
void pkg_send(char header, packet_t packet) {
    /* on packet:
     * -------------------------- ---------------------------- -----------------------
     * | Length | CMD | DATA ... | Length | CMD | INFORMATION |Length | CMD | ... ... |
     * -------------------------- ---------------------------- -----------------------
     *    1        2 -> length    length+1 length+2 length+3   ....
     */

    while ((U1STAbits.TRMT == 0) && (DMA1CONbits.CHEN == 0));

    int i;
    BufferTx[0] = header;
    BufferTx[1] = packet.length;

    for (i = 0; i < packet.length; i++) {
        BufferTx[i + HEAD_PKG] = packet.buffer[i];
    }

    BufferTx[packet.length + HEAD_PKG] = pkg_checksum(BufferTx, HEAD_PKG, packet.length + HEAD_PKG);

    DMA1CNT = (HEAD_PKG + packet.length + 1) - 1; // # of DMA requests
    DMA1CONbits.CHEN = 1; // Enable DMA1 Channel
    DMA1REQbits.FORCE = 1; // Manual mode: Kick-start the 1st transfer
}
