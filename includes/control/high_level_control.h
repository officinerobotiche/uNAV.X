/* 
 * File:   highlevelcontrol.h
 * Author: raffaello
 *
 * Created on 13 giugno 2013, 18.59
 */

#ifndef HIGHLEVELCONTROL_H
#define	HIGHLEVELCONTROL_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "packet/packet.h"

    void init_coordinate(void);
    void update_coord(void);
    int deadReckoning(void);
    int odometry(coordinate_t delta);

#ifdef	__cplusplus
}
#endif

#endif	/* HIGHLEVELCONTROL_H */

