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

/******************************************************************************/
/* Configuration Bits                                                         */
/*                                                                            */
/* This is not all available configuration bits for all dsPIC devices.        */
/* Refer to the dsPIC device specific .h file in the compiler                 */
/* support\dsPIC33F\h directory for complete options specific to the device   */
/* selected.  For additional information about what hardware configurations   */
/* mean in terms of device operation, refer to the device datasheet           */
/* 'Special Features' chapter.                                                */
/*                                                                            */
/* A feature of MPLAB X is the 'Generate Source Code to Output' utility in    */
/* the Configuration Bits window.  Under Window > PIC Memory Views >          */
/* Configuration Bits, a user controllable configuration bits window is       */
/* available to Generate Configuration Bits source code which the user can    */
/* paste into this project.                                                   */
/******************************************************************************/

/** Oscillator selection configuration
 * * FNOSC_PRI -> Primary (XT, HS, EC) Oscillator
 * * IESO_ON -> Start-up device with FRC, then automatically switch to
 * user-selected oscillator source when ready
 */
_FOSCSEL(FNOSC_PRI & IESO_ON);

/** Oscillator configuration
 * * FCKSM_CSECME -> Both Clock Switching and Fail-Safe Clock Monitor are enabled
 * * OSCIOFNC_OFF -> OSC2 pin has clock out function
 * * POSCMD_HS -> Primary Oscillator Mode, HS Crystal
*/
_FOSC(FCKSM_CSECME & OSCIOFNC_OFF & POSCMD_HS);

/** Watchdog Timer Enabled/disabled by user software
 * (LPRC can be disabled by clearing SWDTEN bit in RCON register
 */
_FWDT(FWDTEN_OFF);

_FICD(JTAGEN_OFF & ICS_PGD1);
_FGS(GSS_OFF & GCP_OFF & GWRP_OFF);