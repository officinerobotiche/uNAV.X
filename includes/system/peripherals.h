/*
 * Copyright (C) 2015 Officine Robotiche
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

#ifndef PERIPHERALS_H
#define	PERIPHERALS_H

#ifdef	__cplusplus
extern "C" {
#endif
    
//#define ADC_HIGH_FREQ

#ifdef ADC_HIGH_FREQ
#define ADC_BUFF 8
#else
#define ADC_BUFF 16
#endif
    
#define ADC_BUFF_PIN (ADC_BUFF / 4)

    /**
     * I/O and Peripheral Initialization
     */
    void Peripherals_Init(void);


#ifdef	__cplusplus
}
#endif

#endif	/* PERIPHERALS_H */

