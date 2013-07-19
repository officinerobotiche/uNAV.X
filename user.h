/******************************************************************************/
/* User Level #define Macros                                                  */
/******************************************************************************/

#define LED _LATA4              // Led Blu
#define MOTOR_ENABLE1 _LATB2    // Enable Motore 1
#define MOTOR_ENABLE2 _LATB3    // Enable Motore 2

#include <stddef.h>

/******************************************************************************/
/* User Function Prototypes                                                   */
/******************************************************************************/

/* TODO User level functions prototypes (i.e. InitApp) go here */

void InitApp(void);         /* I/O and Peripheral Initialization */
void protectedMemcpy(unsigned reg, void *destination, const void *source, size_t num); /* protected memcpy */
int maxValue(float myArray[], size_t size);