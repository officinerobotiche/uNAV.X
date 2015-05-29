/*
 * Copyright (C) 2014 Officine Robotiche
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

#ifndef USER_H
#define	USER_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <stddef.h>

/******************************************************************************/
/* User Level #define Macros                                                  */
/******************************************************************************/

/******************************************************************************/
/* User Function Prototypes                                                   */
/******************************************************************************/

    /**
     * I/O and Peripheral Initialization
     */
    void InitApp(void);
    /**
     * Protected memcpy, stop particular interrupt and copy data.
     * @param reg Interrupt to disable
     * @param destination data
     * @param source data
     * @param num size of data
     */
    void protectedMemcpy(unsigned reg, void *destination, const void *source, size_t num);

#ifdef	__cplusplus
}
#endif

#endif	/* USER_H */