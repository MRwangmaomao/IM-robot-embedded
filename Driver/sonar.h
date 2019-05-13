#ifdef __cplusplus
extern "C" {
#endif
#ifndef _SONAR_H_
#define _SONAR_H_

#include "config.h"
extern uint8_t distance_sonars[4];

void sonar_init(void);
void read_distances(void);

#endif //_SONAR_H_

#ifdef __cplusplus
}
#endif
