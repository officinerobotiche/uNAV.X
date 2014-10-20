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

_FOSCSEL(FNOSC_PRI); /* Primary (XT, HS, EC) Oscillator */
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_XT);
/** Clock Switching is enabled and Fail Safe Clock Monitor is disabled
 * OSC2 Pin Function: OSC2 is Clock Output
 * Primary Oscillator Mode : XT Crystal
 */
_FWDT(FWDTEN_OFF);
/** Watchdog Timer Enabled/disabled by user software
 * (LPRC can be disabled by clearing SWDTEN bit in RCON register
 */
_FICD(JTAGEN_OFF & ICS_PGD1);
_FGS(GSS_OFF & GCP_OFF & GWRP_OFF);
