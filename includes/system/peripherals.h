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

        // Current ADC buffer dimension
#define ADC_CHANNELS 2
#define ADC_BUFF 64
#define TOT_ADC_BUFF ADC_CHANNELS * ADC_BUFF

typedef int ADC[ADC_CHANNELS][ADC_BUFF];

#ifdef UNAV_V1
/// Number of available LEDs
#define LED_NUM 4
/// LED 1 - Green
#define LED1_BIT _LATC6          // Led 1 Green
#define LED1 0                   // Led 1 Green
/// LED 2 - Red
#define LED2_BIT _LATC7          // Led 2 Red
#define LED2 1                   // Led 2 Red
/// LED 3 - Yellow
#define LED3_BIT _LATC8          // Led 3 Yellow
#define LED3 2                   // Led 3 Yellow
/// LED 4 - Blue
#define LED4_BIT _LATC9          // Led 4 Blue
#define LED4 3                   // Led 4 Blue
#elif ROBOCONTROLLER_V3
/// Number of available LEDs
#define LED_NUM 2
/// LED 1 - Green
#define LED1_BIT _LATA8          // Led 1 green
#define LED1 0                   // Led 1 green
/// LED 2 - Green
#define LED2_BIT _LATA9          // Led 2 green
#define LED2 1                   // Led 2 green
#elif MOTION_CONTROL
/// Number of available LEDs
#define LED_NUM 1
/// LED 1 - Green
#define LED1_BIT _LATA4          // Led Blue
#define LED1 0                   // Led Blue
#endif

    /**
     * I/O and Peripheral Initialization
     */
    void Peripherals_Init(void);
    /**
     * Initialization led blink
     */
    void InitLEDs(void);
    /**
     * Update frequency or type of blink
     * @param led to control
     * @param blink number of blinks
     */
    inline void UpdateBlink(short num, short blink);


#ifdef	__cplusplus
}
#endif

#endif	/* PERIPHERALS_H */

