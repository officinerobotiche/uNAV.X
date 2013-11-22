/******************************************************************************/
/* User Level #define Macros                                                  */
/******************************************************************************/

#define LED _LATA4              // Led Blu
#define MOTOR_ENABLE1 _LATB2    // Enable Motore 1
#define MOTOR_ENABLE2 _LATB3    // Enable Motore 2
#define PID_FLAG IFS0bits.OC1IF
#define PARSER_FLAG IFS0bits.OC2IF
#define DEAD_RECKONING_FLAG IFS3bits.RTCIF

#define SGN(x)  ( ((x) < 0) ?  -1 : ( ((x) == 0 ) ? 0 : 1) )

#include <stddef.h>

/******************************************************************************/
/* User Function Prototypes                                                   */
/******************************************************************************/

/* TODO User level functions prototypes (i.e. InitApp) go here */

void InitApp(void); /* I/O and Peripheral Initialization */
void protectedMemcpy(unsigned reg, void *destination, const void *source, size_t num); /* protected memcpy */
int maxValue(float myArray[], size_t size);