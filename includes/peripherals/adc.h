/*
 * Copyright (C) 2014-2017 Officine Robotiche
 * Author: Raffaello Bonghi
 * email:  raffaello.bonghi@officinerobotiche.it
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
#ifndef ADC_H
#define	ADC_H

#ifdef	__cplusplus
extern "C" {
#endif

/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/
    
#include <or_peripherals/GPIO/adc.h>
    
/******************************************************************************/
/* System Level #define Macros                                                */
/******************************************************************************/

//#define ADC_HIGH_FREQ

#ifdef ADC_HIGH_FREQ
#define ADC_BUFF 8
#else
#define ADC_BUFF 16
#endif
    
#define ADC_BUFF_PIN (ADC_BUFF / 4)
    
/******************************************************************************/
/* System Function Prototypes                                                 */
/******************************************************************************/
/**
 * Initialization DMA for ADC
 */
void InitDMA0(void);
/**
 * Initialization ADC with 2 simultaneous channels
 */
void InitADC_2Sim(void);
/**
 * Initialization ADC with 4 simultaneous channels
 */
void InitADC_4Sim(void);

#ifdef	__cplusplus
}
#endif

#endif	/* ADC_H */

