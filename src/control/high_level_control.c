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

#include <stdint.h>          /* For uint16_t definition                       */
#include <stdbool.h>         /* For true/false definition                     */
#include <dsp.h>
#include <string.h>
#include <float.h>

#include "control/high_level_control.h"
#include "control/motors_PID.h"
#include "communication/serial.h"
#include "communication/parsing_packet.h"

coordinate_t coordinate;
delta_odometry_t delta_odometry;
unsigned int counter_delta = 0;
bool autosend_delta_odometry = false;

float sinTh_old = 0, cosTh_old = 1;

// From motors PID
extern volatile parameter_motors_t parameter_motors;
extern volatile int PulsEncL, PulsEncR;
extern k_odo_t k_odo;
extern float wheel_m;

/******************************************************************************/
/* Dead Reckoning functions                                                   */

/******************************************************************************/

void init_coordinate(void) {
    coordinate.x = 0;
    coordinate.y = 0;
    coordinate.theta = 0;
    coordinate.space = 0;
}

void update_coord(void) {
    sinTh_old = sinf(coordinate.theta);
    cosTh_old = cosf(coordinate.theta);
}

int deadReckoning(void) {
    unsigned int t = TMR1; // Timing function
    volatile coordinate_t delta;
    float WheelSpL = k_odo.k_left * PulsEncL; // Spostamento Ruota sinistra
    float WheelSpR = k_odo.k_right * PulsEncR; // Spostamento Ruota destra
    float SumSp = WheelSpR + WheelSpL; // Calcolo della somma degli spostamenti delle ruote
    float DifSp = WheelSpR - WheelSpL; // Calcolo della differenza degli spostamenti delle ruote
    PulsEncL = 0; // Flush variabile
    PulsEncR = 0; // Flush variabile

    if (fabs(DifSp) <= parameter_motors.sp_min) {
        delta.theta = 0;
        delta.space = WheelSpR;
        delta.x = delta.space * cosTh_old;
        delta.y = delta.space * sinTh_old;
    } else if (fabs(SumSp) <= parameter_motors.sp_min) {
        delta.theta = DifSp / parameter_motors.wheelbase;
        coordinate.theta = fmodf(coordinate.theta + delta.theta, 2 * PI); // Angolo normalizzato tra [0,2*PI]
        sinTh_old = sinf(coordinate.theta);
        cosTh_old = cosf(coordinate.theta);
        delta.x = 0;
        delta.y = 0;
        delta.space = 0;
    } else {
        delta.theta = DifSp / parameter_motors.wheelbase;
        coordinate.theta = fmodf(coordinate.theta + delta.theta, 2 * PI); // Angolo normalizzato tra [0,2*PI]
        float cosTh_new = cosf(coordinate.theta);
        float sinTh_new = sinf(coordinate.theta);
        delta.space = SumSp / 2;
        float radius = wheel_m * (SumSp / DifSp);

        delta.x = radius * (sinTh_new - sinTh_old);
        delta.y = radius * (cosTh_old - cosTh_new);
        sinTh_old = sinTh_new;
        cosTh_old = cosTh_new;
    }

    if (autosend_delta_odometry) {
        // Add delta step in buffer
        delta_odometry.delta[counter_delta] = delta;
        counter_delta++;
        if (counter_delta == BUFFER_ODOMETRY) {
            abstract_packet_t packet;
            packet.delta_odometry = delta_odometry;
            packet_t send = encoderSingle(createDataPacket(DELTA_ODOMETRY, HASHMAP_MOTION, &packet));
            pkg_send(HEADER_ASYNC, send);
            counter_delta = 0;
        }
    }

    // Calculate odometry
    odometry(delta);

    return TMR1 - t; // Time of esecution
}

int odometry(coordinate_t delta) {
    unsigned int t = TMR1; // Timing function

    coordinate.space += delta.space;
    coordinate.x += delta.x;
    coordinate.y += delta.y;

    return TMR1 - t; // Time of esecution
}