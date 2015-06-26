/*
 * Copyright (C) 2014 Officine Robotiche
 * Authors: Guido Ottaviani, Raffaello Bonghi
 * email:  guido@guiott.com, raffaello.bonghi@officinerobotiche.it
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
#include <system/events.h>
    
/******************************************************************************/
/* System Level #define Macros                                                */
/******************************************************************************/
    
    #define I2C_SDA    _LATB9    //  from _RA3, _RA2, mods per Bill P.
    #define I2C_SCL    _LATB8
    #define _I2CEN  I2C1CONbits.I2CEN

    #define I2CBRGVAL ( (int)(((1/100e3) - 130E-9) * FCY)-2 ) // 392 // 100 Khz

/******************************************************************************/
/* System Function Prototypes                                                 */
/******************************************************************************/

    /**
     * Trigger the I2C service routine to run at low priority in libUDB.c
     * initialize the I2C peripheral
     */
    void Init_I2C(void);

#ifdef	__cplusplus
}
#endif

#endif	/* I2C_H */

